"""Exact palette decoding, using per-channel LUTs when colors permit it."""
import ctypes
import os
from pathlib import Path
import numpy as np


def native_decoder():
    try:
        from ament_index_python.packages import get_package_prefix, PackageNotFoundError
    except ImportError:
        return None
    try:
        prefix=Path(get_package_prefix('evtol_sensor_native'))
    except PackageNotFoundError:
        return None
    path=prefix/('bin/evtol_sensor_native.dll' if os.name=='nt' else 'lib/libevtol_sensor_native.so')
    if not path.exists():
        return None
    library=ctypes.CDLL(str(path))
    function=library.evtol_decode_bgr
    function.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_size_t]
    function.restype=None
    return function


class SemanticDecoder:
    def __init__(self, classes, unknown=255, prefer_opencv=True, prefer_native=True):
        self.unknown = unknown
        colors = np.asarray([item['color_rgb'][::-1] for item in classes], dtype=np.uint8)
        self.cv2 = None
        self.native = native_decoder() if prefer_native else None
        if not self.native and prefer_opencv and all(len(set(colors[:, i])) == len(classes) for i in range(3)):
            try:
                import cv2
                cv2.setNumThreads(1)
                self.cv2 = cv2
            except ImportError:
                pass
        if self.cv2:
            self.lut = np.full((256, 1, 3), unknown, np.uint8)
            for color, item in zip(colors, classes):
                for channel, value in enumerate(color):
                    self.lut[value, 0, channel] = item['id']
        else:
            self.lut = np.full((256, 256, 256), unknown, np.uint8)
            for color, item in zip(colors, classes):
                self.lut[tuple(color)] = item['id']

    def __call__(self, image):
        if image.dtype!=np.uint8 or image.ndim!=3 or image.shape[2]!=3:
            raise ValueError('Expected uint8 BGR image')
        if self.native:
            image=np.ascontiguousarray(image)
            labels=np.empty(image.shape[:2],dtype=np.uint8)
            self.native(image.ctypes.data,labels.ctypes.data,self.lut.ctypes.data,labels.size)
            return labels
        if not self.cv2:
            return self.lut[image[:, :, 0], image[:, :, 1], image[:, :, 2]]
        mapped = self.cv2.LUT(image, self.lut)
        labels = mapped[:, :, 0].copy()
        # All three channel values must identify the same palette color.
        labels[(mapped[:, :, 1] != labels) | (mapped[:, :, 2] != labels)] = self.unknown
        return labels
