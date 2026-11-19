"""Reader for the AirSim Windows shared-memory BGR8 camera ring."""
from __future__ import annotations

import mmap
import struct
import time
from dataclasses import dataclass


MAGIC = 0x4D485341
VERSION = 1
PIXEL_FORMAT_BGR8 = 1
RING_HEADER = struct.Struct("<IHHIIIIIIIIQqQ")
SLOT_HEADER = struct.Struct("<qQQIIIII20x")


def aligned_slot_size(frame_bytes: int) -> int:
    return (SLOT_HEADER.size + int(frame_bytes) + 63) & ~63


def expected_mapping_size(width: int, height: int, slots: int) -> int:
    frame_bytes = int(width) * int(height) * 3
    return RING_HEADER.size + int(slots) * aligned_slot_size(frame_bytes)


@dataclass(frozen=True)
class SharedMemoryFrame:
    frame_id: int
    timestamp_ns: int
    width: int
    height: int
    stride: int
    image_type: int
    bgr_bytes: bytes


class SharedMemoryCameraReader:
    def __init__(
        self,
        mapping_name: str,
        width: int,
        height: int,
        slots: int = 8,
    ) -> None:
        self.mapping_name = str(mapping_name)
        self.width = int(width)
        self.height = int(height)
        self.slots = int(slots)
        self.mapping_size = expected_mapping_size(
            self.width, self.height, self.slots
        )
        try:
            self.mapping = mmap.mmap(
                -1,
                self.mapping_size,
                tagname=self.mapping_name,
                access=mmap.ACCESS_READ,
            )
        except TypeError as exc:
            raise RuntimeError("共享内存相机当前只支持 Windows") from exc
        self._validate_header()

    def _read_header(self) -> tuple[int, ...]:
        return RING_HEADER.unpack_from(self.mapping, 0)

    def _validate_header(self) -> None:
        (
            magic,
            version,
            header_size,
            slot_count,
            slot_header_size,
            width,
            height,
            channels,
            stride,
            pixel_format,
            slot_size,
            mapping_size,
            _latest_frame_id,
            _reserved,
        ) = self._read_header()
        expected_stride = self.width * 3
        expected_slot_size = aligned_slot_size(expected_stride * self.height)
        if magic != MAGIC or version != VERSION:
            raise RuntimeError(
                f"共享内存 {self.mapping_name!r} 头无效；"
                "请确认已重新编译并启动共享内存版 AirSim 插件"
            )
        expected = (
            RING_HEADER.size,
            self.slots,
            SLOT_HEADER.size,
            self.width,
            self.height,
            3,
            expected_stride,
            PIXEL_FORMAT_BGR8,
            expected_slot_size,
            self.mapping_size,
        )
        actual = (
            header_size,
            slot_count,
            slot_header_size,
            width,
            height,
            channels,
            stride,
            pixel_format,
            slot_size,
            mapping_size,
        )
        if actual != expected:
            raise RuntimeError(
                f"共享内存布局不一致：actual={actual}, expected={expected}"
            )

    def latest_frame_id(self) -> int:
        """Return the producer's latest published frame id."""
        return int(self._read_header()[12])

    def _try_read_frame(self, frame_id: int) -> SharedMemoryFrame | None:
        header = self._read_header()
        latest_frame_id = int(header[12])
        if latest_frame_id < frame_id:
            return None
        if latest_frame_id - frame_id >= self.slots:
            raise RuntimeError(
                f"共享内存帧 {frame_id} 已被覆盖；当前最新帧为 {latest_frame_id}，"
                f"环形缓冲区只有 {self.slots} 个槽"
            )

        slot_size = int(header[10])
        slot_offset = RING_HEADER.size + (frame_id % self.slots) * slot_size
        guard_before = struct.unpack_from("<q", self.mapping, slot_offset)[0]
        if guard_before & 1:
            return None

        (
            _guard,
            actual_frame_id,
            timestamp_ns,
            width,
            height,
            stride,
            data_size,
            image_type,
        ) = SLOT_HEADER.unpack_from(self.mapping, slot_offset)
        if actual_frame_id != frame_id:
            return None
        expected_data_size = int(stride) * int(height)
        if data_size != expected_data_size:
            raise RuntimeError(
                f"共享内存帧长度 {data_size} 与 {expected_data_size} 不一致"
            )
        payload_offset = slot_offset + SLOT_HEADER.size
        bgr_bytes = bytes(
            self.mapping[payload_offset : payload_offset + data_size]
        )
        guard_after = struct.unpack_from("<q", self.mapping, slot_offset)[0]
        if guard_before != guard_after or guard_after & 1:
            return None
        return SharedMemoryFrame(
            frame_id=int(actual_frame_id),
            timestamp_ns=int(timestamp_ns),
            width=int(width),
            height=int(height),
            stride=int(stride),
            image_type=int(image_type),
            bgr_bytes=bgr_bytes,
        )

    def wait_for_new_frame(
        self,
        after_frame_id: int,
        timeout_s: float,
        poll_interval_s: float = 0.0005,
    ) -> SharedMemoryFrame:
        """Copy the next resident frame; start at latest when attaching."""
        deadline = time.perf_counter() + float(timeout_s)
        while time.perf_counter() < deadline:
            latest_frame_id = self.latest_frame_id()
            if latest_frame_id > int(after_frame_id):
                # Drain still-resident frames in order. Taking only latest loses
                # valid captures when two frames arrive during one ROS publish.
                next_id = (latest_frame_id if after_frame_id <= 0 else
                           max(after_frame_id + 1, latest_frame_id - self.slots + 1))
                frame = self._try_read_frame(next_id)
                if frame is not None:
                    return frame
            time.sleep(float(poll_interval_s))
        raise TimeoutError(
            f"等待共享内存 {self.mapping_name!r} 的新帧超时："
            f"last={after_frame_id}, latest={self.latest_frame_id()}"
        )

    def read_frame(
        self,
        frame_id: int,
        timeout_s: float,
        poll_interval_s: float = 0.0005,
    ) -> SharedMemoryFrame:
        """Wait for and copy one exact frame, used to align RGB and semantics."""
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            frame = self._try_read_frame(int(frame_id))
            if frame is not None:
                return frame
            time.sleep(float(poll_interval_s))
        raise TimeoutError(
            f"等待共享内存 {self.mapping_name!r} 的帧 {frame_id} 超时；"
            f"latest={self.latest_frame_id()}"
        )

    def read_latest(
        self,
        expected_frame_id: int | None = None,
        retries: int = 50,
    ) -> SharedMemoryFrame:
        for _ in range(int(retries)):
            header = self._read_header()
            latest_frame_id = int(header[12])
            if latest_frame_id <= 0 or (
                expected_frame_id is not None
                and latest_frame_id < int(expected_frame_id)
            ):
                time.sleep(0)
                continue

            slot_size = int(header[10])
            slot_offset = RING_HEADER.size + (
                latest_frame_id % self.slots
            ) * slot_size
            guard_before = struct.unpack_from("<q", self.mapping, slot_offset)[0]
            if guard_before & 1:
                time.sleep(0)
                continue

            (
                _guard,
                frame_id,
                timestamp_ns,
                width,
                height,
                stride,
                data_size,
                image_type,
            ) = SLOT_HEADER.unpack_from(self.mapping, slot_offset)
            if frame_id != latest_frame_id:
                continue
            expected_data_size = int(stride) * int(height)
            if data_size != expected_data_size:
                raise RuntimeError(
                    f"共享内存帧长度 {data_size} 与 {expected_data_size} 不一致"
                )
            payload_offset = slot_offset + SLOT_HEADER.size
            bgr_bytes = bytes(
                self.mapping[payload_offset : payload_offset + data_size]
            )
            guard_after = struct.unpack_from("<q", self.mapping, slot_offset)[0]
            if guard_before == guard_after and not (guard_after & 1):
                return SharedMemoryFrame(
                    frame_id=int(frame_id),
                    timestamp_ns=int(timestamp_ns),
                    width=int(width),
                    height=int(height),
                    stride=int(stride),
                    image_type=int(image_type),
                    bgr_bytes=bgr_bytes,
                )
        raise RuntimeError(
            f"读取共享内存 {self.mapping_name!r} 时连续 {retries} 次发生覆盖或数据未就绪"
        )

    def close(self) -> None:
        mapping = getattr(self, "mapping", None)
        if mapping is not None:
            mapping.close()
            self.mapping = None

    def __enter__(self) -> "SharedMemoryCameraReader":
        return self

    def __exit__(self, *_args) -> None:
        self.close()
