"""Windows shared-memory ring for LiDAR XYZ float32 frames."""
from __future__ import annotations

import mmap
import struct
from dataclasses import dataclass

MAGIC = 0x4D534C44
VERSION = 1
RING_HEADER = struct.Struct("<IHHIIQQ")
SLOT_HEADER = struct.Struct("<qQQII8x")

def _slot_size(max_points: int) -> int:
    return (SLOT_HEADER.size + int(max_points) * 12 + 63) & ~63

def mapping_size(max_points: int, slots: int) -> int:
    return RING_HEADER.size + int(slots) * _slot_size(max_points)

@dataclass(frozen=True)
class LidarSharedFrame:
    frame_id: int
    timestamp_ns: int
    xyz_bytes: bytes
    point_count: int
    scan_complete: bool = False

class LidarSharedMemory:
    def __init__(self, name: str, max_points: int, slots: int = 8, create: bool = False):
        self.name, self.max_points, self.slots = str(name), int(max_points), int(slots)
        self.mapping = mmap.mmap(-1, mapping_size(self.max_points, self.slots), tagname=self.name,
                                 access=mmap.ACCESS_WRITE if create else mmap.ACCESS_READ)
        if create:
            RING_HEADER.pack_into(self.mapping, 0, MAGIC, VERSION, RING_HEADER.size, self.slots,
                                  self.max_points, 0, 0)
        else:
            actual = RING_HEADER.unpack_from(self.mapping, 0)
            if actual[:5] != (MAGIC, VERSION, RING_HEADER.size, self.slots, self.max_points):
                raise RuntimeError(f"LiDAR shared-memory layout mismatch: {actual[:5]}")

    def latest_id(self) -> int:
        # Header fields are magic, version, size, slots, max_points,
        # reserved, latest_frame_id; the latest id is the seventh field.
        return int(RING_HEADER.unpack_from(self.mapping, 0)[6])

    def publish(self, frame_id: int, timestamp_ns: int, xyz_bytes: bytes, point_count: int,
                scan_complete: bool = False) -> None:
        count = min(int(point_count), self.max_points)
        slot = RING_HEADER.size + (frame_id % self.slots) * _slot_size(self.max_points)
        old = struct.unpack_from("<q", self.mapping, slot)[0]
        seq = abs(old) | 1
        flags = int(scan_complete and int(point_count) <= self.max_points)
        struct.pack_into("<qQQII", self.mapping, slot, seq, frame_id, timestamp_ns, count, flags)
        start = slot + SLOT_HEADER.size
        self.mapping[start:start + count * 12] = xyz_bytes[:count * 12]
        struct.pack_into("<q", self.mapping, slot, seq + 1)
        struct.pack_into("<Q", self.mapping, 24, frame_id)

    def read_latest(self, after_id: int = 0) -> LidarSharedFrame | None:
        latest = self.latest_id()
        return self._read(latest, after_id)

    def read_next(self, after_id: int = 0) -> LidarSharedFrame | None:
        latest = self.latest_id()
        target = latest if after_id <= 0 else max(after_id+1, latest-self.slots+1)
        return self._read(min(target, latest), after_id)

    def _read(self, latest: int, after_id: int) -> LidarSharedFrame | None:
        if latest <= int(after_id):
            return None
        slot = RING_HEADER.size + (latest % self.slots) * _slot_size(self.max_points)
        before = struct.unpack_from("<q", self.mapping, slot)[0]
        if before & 1:
            return None
        _guard, frame_id, timestamp_ns, count, flags = SLOT_HEADER.unpack_from(self.mapping, slot)
        if frame_id != latest or count > self.max_points:
            return None
        start = slot + SLOT_HEADER.size
        payload = bytes(self.mapping[start:start + count * 12])
        after = struct.unpack_from("<q", self.mapping, slot)[0]
        if before != after or after & 1:
            return None
        return LidarSharedFrame(int(frame_id), int(timestamp_ns), payload, int(count), bool(flags & 1))

    def close(self) -> None:
        self.mapping.close()
