"""AirSim LiDAR 到相机图像的几何投影与语义着色。"""
from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SensorPose:
    """传感器相对机体的 AirSim NED 位姿（米、角度）。"""

    xyz_m: tuple[float, float, float]
    rpy_deg: tuple[float, float, float]

    @classmethod
    def from_config(cls, value: Mapping[str, Sequence[float]]) -> SensorPose:
        return cls(
            tuple(float(item) for item in value["xyz_m"]),
            tuple(float(item) for item in value["rpy_deg"]),
        )


@dataclass(frozen=True)
class ProjectionResult:
    """投影成功的点在原点云中的索引、像素和相机前向深度。"""

    point_indices: np.ndarray
    pixels_uv: np.ndarray
    depth_m: np.ndarray


def rotation_matrix_rpy_deg(rpy_deg: Sequence[float]) -> np.ndarray:
    """返回 AirSim NED 下从传感器局部系到机体系的旋转矩阵。"""
    roll, pitch, yaw = np.deg2rad(np.asarray(rpy_deg, dtype=np.float64))
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)))
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)))
    rz = np.array(((cy, -sy, 0), (sy, cy, 0), (0, 0, 1)))
    return rz @ ry @ rx


def quaternion_xyzw_to_rotation_matrix(
    quaternion_xyzw: Sequence[float],
) -> np.ndarray:
    """Return the body-to-NED rotation represented by an AirSim quaternion."""
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    if quaternion.shape != (4,):
        raise ValueError("quaternion_xyzw must contain four values")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12 or not np.isfinite(norm):
        raise ValueError("quaternion_xyzw must be finite and non-zero")
    x, y, z, w = quaternion / norm
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=np.float64,
    )


def ned_to_enu_points(points_ned: np.ndarray) -> np.ndarray:
    """Convert NED points/vectors to ENU (east, north, up)."""
    points = np.asarray(points_ned, dtype=np.float64)
    if points.ndim < 1 or points.shape[-1] != 3:
        raise ValueError("NED points must end in a three-value coordinate axis")
    return np.stack((points[..., 1], points[..., 0], -points[..., 2]), axis=-1)


def enu_to_ned_points(points_enu: np.ndarray) -> np.ndarray:
    """Convert ENU points/vectors to AirSim's NED convention."""
    points = np.asarray(points_enu, dtype=np.float64)
    if points.ndim < 1 or points.shape[-1] != 3:
        raise ValueError("ENU points must end in a three-value coordinate axis")
    return np.stack((points[..., 1], points[..., 0], -points[..., 2]), axis=-1)


def camera_intrinsics_from_hfov(
    width: int,
    height: int,
    horizontal_fov_deg: float,
) -> np.ndarray:
    """由 AirSim 水平 FOV 构建零畸变、方形像素内参。"""
    if width <= 0 or height <= 0:
        raise ValueError("图像宽高必须大于 0")
    if not 0.0 < float(horizontal_fov_deg) < 180.0:
        raise ValueError("相机水平 FOV 必须在 0..180 度之间")
    focal = width / (2.0 * np.tan(np.deg2rad(horizontal_fov_deg) / 2.0))
    return np.array(
        ((focal, 0.0, (width - 1.0) / 2.0),
         (0.0, focal, (height - 1.0) / 2.0),
         (0.0, 0.0, 1.0)),
        dtype=np.float64,
    )


def lidar_to_camera_points(
    points_lidar_xyz: np.ndarray,
    lidar_pose: SensorPose,
    camera_pose: SensorPose,
) -> np.ndarray:
    """把 SensorLocalFrame LiDAR 点变换到 AirSim 相机局部 NED 轴。"""
    points = np.asarray(points_lidar_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("LiDAR 点形状必须为 Nx3")
    lidar_rotation = rotation_matrix_rpy_deg(lidar_pose.rpy_deg)
    camera_rotation = rotation_matrix_rpy_deg(camera_pose.rpy_deg)
    lidar_translation = np.asarray(lidar_pose.xyz_m, dtype=np.float64)
    camera_translation = np.asarray(camera_pose.xyz_m, dtype=np.float64)
    points_vehicle = points @ lidar_rotation.T + lidar_translation
    return (points_vehicle - camera_translation) @ camera_rotation


def project_lidar_to_image(
    points_lidar_xyz: np.ndarray,
    width: int,
    height: int,
    horizontal_fov_deg: float,
    lidar_pose: SensorPose,
    camera_pose: SensorPose,
    min_depth_m: float = 0.1,
    max_depth_m: float | None = None,
) -> ProjectionResult:
    """投影 LiDAR 点；AirSim 相机轴为 X 前、Y 右、Z 下。"""
    points_camera = lidar_to_camera_points(
        points_lidar_xyz, lidar_pose, camera_pose
    )
    depth = points_camera[:, 0]
    valid = np.isfinite(points_camera).all(axis=1) & (depth >= min_depth_m)
    if max_depth_m is not None:
        valid &= depth <= max_depth_m
    candidate_indices = np.flatnonzero(valid)
    if not len(candidate_indices):
        return ProjectionResult(
            candidate_indices,
            np.empty((0, 2), dtype=np.int32),
            np.empty(0, dtype=np.float32),
        )

    camera_points = points_camera[candidate_indices]
    intrinsics = camera_intrinsics_from_hfov(width, height, horizontal_fov_deg)
    u = intrinsics[0, 0] * camera_points[:, 1] / camera_points[:, 0] + intrinsics[0, 2]
    v = intrinsics[1, 1] * camera_points[:, 2] / camera_points[:, 0] + intrinsics[1, 2]
    pixels = np.rint(np.column_stack((u, v))).astype(np.int32)
    inside = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < height)
    )
    return ProjectionResult(
        candidate_indices[inside],
        pixels[inside],
        camera_points[inside, 0].astype(np.float32, copy=False),
    )


def assign_semantic_labels(
    point_count: int,
    projection: ProjectionResult,
    class_mask: np.ndarray,
    unknown_class_id: int = -1,
) -> np.ndarray:
    """按投影像素给原始点云生成一一对应的 int32 类别数组。"""
    mask = np.asarray(class_mask)
    if mask.ndim != 2:
        raise ValueError("class_mask 必须是 HxW 单通道数组")
    labels = np.full(point_count, int(unknown_class_id), dtype=np.int32)
    if len(projection.point_indices):
        u = projection.pixels_uv[:, 0]
        v = projection.pixels_uv[:, 1]
        labels[projection.point_indices] = mask[v, u].astype(np.int32)
    return labels


