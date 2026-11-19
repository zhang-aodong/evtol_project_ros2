"""Ultralytics YOLO26 实例分割的轻量实时适配器。"""
from __future__ import annotations

import logging
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SegmentationFrame:
    timestamp_ns: int
    class_mask: np.ndarray
    confidence_mask: np.ndarray
    class_names: dict[int, str]
    palette_rgb: dict[int, tuple[int, int, int]]
    inference_ms: float
    prediction_ms: float = 0.0


def default_class_color_rgb(class_id: int) -> tuple[int, int, int]:
    """稳定的高区分度类别颜色，不依赖 Python hash 随机种子。"""
    base = (
        (255, 56, 56), (56, 255, 56), (56, 56, 255),
        (255, 178, 29), (255, 56, 255), (56, 255, 255),
        (151, 56, 255), (255, 157, 151), (112, 105, 191),
        (89, 121, 72), (190, 225, 64), (206, 190, 59),
    )
    return base[int(class_id) % len(base)]


def compose_instance_masks(
    masks: np.ndarray,
    class_ids: Sequence[int],
    confidences: Sequence[float],
    output_shape: tuple[int, int],
    class_id_map: Mapping[int, int] | None = None,
    background_class_id: int = -1,
) -> tuple[np.ndarray, np.ndarray]:
    """以逐像素最高实例置信度把实例掩码合成为语义掩码。"""
    height, width = output_shape
    class_mask = np.full((height, width), background_class_id, dtype=np.int32)
    confidence_mask = np.zeros((height, width), dtype=np.float32)
    values = np.asarray(masks)
    if values.size == 0:
        return class_mask, confidence_mask
    if values.ndim != 3:
        raise ValueError("YOLO masks 必须是 NxHxW")
    for mask, source_class_id, confidence in zip(values, class_ids, confidences):
        if mask.shape != (height, width):
            source_height, source_width = mask.shape
            # Instance masks are still in the letterboxed inference image.
            gain = min(source_height / height, source_width / width)
            pad_x = (source_width - round(width * gain)) / 2
            pad_y = (source_height - round(height * gain)) / 2
            top, left = round(pad_y - 0.1), round(pad_x - 0.1)
            bottom = source_height - round(pad_y + 0.1)
            right = source_width - round(pad_x + 0.1)
            mask = mask[top:bottom, left:right]
            source_height, source_width = mask.shape
            y_indices = np.minimum(
                (np.arange(height) * source_height // height), source_height - 1
            )
            x_indices = np.minimum(
                (np.arange(width) * source_width // width), source_width - 1
            )
            mask = np.asarray(mask, dtype=np.float32)[
                y_indices[:, None], x_indices[None, :]
            ]
        active = (mask >= 0.5) & (float(confidence) >= confidence_mask)
        target_id = (
            class_id_map.get(int(source_class_id), int(source_class_id))
            if class_id_map is not None
            else int(source_class_id)
        )
        class_mask[active] = target_id
        confidence_mask[active] = float(confidence)
    return class_mask, confidence_mask


class YoloSegmentationModel:
    """延迟导入 Ultralytics，并把其 Result 转为稳定的项目数据结构。"""

    def __init__(self, settings: Mapping[str, Any], project_root: Path) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "实时分割已启用但缺少 ultralytics；请执行 "
                "pip install -r requirements-perception.txt"
            ) from exc

        weights = Path(str(settings["weights"]))
        if not weights.is_absolute():
            weights = project_root / weights
        if not weights.is_file():
            raise FileNotFoundError(f"YOLO 分割权重不存在：{weights}")
        self.model = YOLO(str(weights))
        self.task = self.model.task
        if self.task not in {"semantic", "segment"}:
            raise RuntimeError(f"需要语义或实例分割模型，实际 task={self.task}")
        requested_device = str(settings.get("device") or "").strip().lower()
        if requested_device in {"gpu", "cuda", "cuda:0"}:
            # Ultralytics expects a CUDA index ("0"), not the generic
            # "gpu"/"cuda" strings.
            requested_device = "0"
        if requested_device and requested_device != "cpu":
            try:
                import torch

                cuda_available = bool(torch.cuda.is_available())
            except Exception:
                cuda_available = False
            if not cuda_available:
                logger.warning(
                    "请求使用 CUDA 设备 %r，但当前 PyTorch 没有可用 CUDA；"
                    "自动回退到 CPU。请安装 CUDA 版 PyTorch。",
                    requested_device,
                )
                requested_device = "cpu"
        self.device = requested_device or None
        self.imgsz = int(settings["imgsz"])
        self.confidence = float(settings["confidence"])
        self.iou = float(settings["iou"])
        self.half = bool(settings["half"])
        # Full-resolution retina masks are substantially slower on CPU. The
        # regular masks are resized to the camera image by compose_instance_masks
        # and are sufficient for LiDAR projection while keeping latency low.
        self.retina_masks = bool(settings.get("retina_masks", False))
        self.background_class_id = int(settings["background_class_id"])
        self.class_id_map = {
            int(source): int(target)
            for source, target in settings.get("class_id_map", {}).items()
        }
        logger.info("分割模型已加载：task=%s，device=%s，classes=%s",
                    self.task, self.device, self.model.names)

    def predict(
        self,
        image_bgr: np.ndarray,
        timestamp_ns: int,
    ) -> SegmentationFrame:
        started = time.perf_counter()
        height, width = image_bgr.shape[:2]
        source = image_bgr
        # Semantic postprocess upsamples *all class logits* before argmax.
        # Bound that work to the model input size, then resize discrete IDs.
        if self.task == "semantic" and max(height, width) > self.imgsz:
            import cv2
            scale = self.imgsz / max(height, width)
            source = cv2.resize(image_bgr, (max(1, round(width * scale)),
                                           max(1, round(height * scale))),
                                interpolation=cv2.INTER_AREA)
        predict_options = dict(
            source=source,
            imgsz=self.imgsz,
            conf=self.confidence,
            iou=self.iou,
            device=self.device,
            retina_masks=self.retina_masks,
            verbose=False,
        )
        # Ultralytics 8.4 已弃用 half 参数；CPU 模式下不传该参数，避免每帧
        # 输出一次弃用警告。旧版/支持 FP16 的 CUDA 环境仍可显式启用。
        if self.half:
            predict_options["half"] = True
        results = self.model.predict(**predict_options)
        if not results:
            raise RuntimeError("YOLO 没有返回推理结果")
        result = results[0]
        semantic = getattr(result, "semantic_mask", None)
        if self.task == "semantic":
            if semantic is None:
                raise RuntimeError("语义分割模型未返回 semantic_mask，不能用空掩码继续飞行")
            class_mask = semantic.data.detach().cpu().numpy().astype(np.int32)
            if class_mask.ndim != 2:
                raise ValueError(f"semantic_mask 必须是 HxW，实际 {class_mask.shape}")
            if class_mask.shape != (height, width):
                import cv2
                class_mask = cv2.resize(class_mask.astype(np.float32), (width, height),
                                        interpolation=cv2.INTER_NEAREST).astype(np.int32)
            original_ids = class_mask.copy() if self.class_id_map else class_mask
            for source_id, target_id in self.class_id_map.items():
                class_mask[original_ids == source_id] = target_id
            # Ultralytics semantic results contain class IDs, not probabilities.
            # Do not invent confidence=1.0 for a dense argmax label.
            confidence_mask = np.full((height, width), np.nan, dtype=np.float32)
        elif result.masks is None or result.boxes is None:
            raw_masks = np.empty((0, height, width), dtype=np.float32)
            source_ids: list[int] = []
            confidences: list[float] = []
        else:
            raw_masks = result.masks.data.detach().cpu().numpy()
            source_ids = result.boxes.cls.detach().cpu().numpy().astype(int).tolist()
            confidences = result.boxes.conf.detach().cpu().numpy().tolist()
        if self.task != "semantic":
            class_mask, confidence_mask = compose_instance_masks(
                raw_masks, source_ids, confidences, (height, width),
                self.class_id_map, self.background_class_id,
            )

        source_names = result.names
        if isinstance(source_names, list):
            source_names = dict(enumerate(source_names))
        class_names: dict[int, str] = {}
        for source_id, name in source_names.items():
            target_id = self.class_id_map.get(int(source_id), int(source_id))
            class_names[target_id] = str(name)
        palette = {
            class_id: default_class_color_rgb(class_id)
            for class_id in class_names
        }
        speed = getattr(result, "speed", {}) or {}
        inference_ms = float(speed.get("inference", 0.0))
        return SegmentationFrame(
            timestamp_ns=timestamp_ns,
            class_mask=class_mask,
            confidence_mask=confidence_mask,
            class_names=class_names,
            palette_rgb=palette,
            inference_ms=inference_ms,
            prediction_ms=(time.perf_counter() - started) * 1000.0,
        )
