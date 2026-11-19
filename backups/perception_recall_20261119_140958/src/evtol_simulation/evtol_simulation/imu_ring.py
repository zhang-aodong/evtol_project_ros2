"""Native IMU sample history; decimate real samples without RPC latest-only loss."""
from collections import deque
import ctypes
import os
import struct

HEADER=struct.Struct('<IHHIIQ40x')
SLOT=struct.Struct('<qQQ10d24x')
MAGIC=0x554D4945


class SharedMemoryImu:
    def __init__(self,name,rate,latency=.1):
        if os.name!='nt':raise RuntimeError('IMU shared memory requires native Windows')
        self.api=ctypes.WinDLL('kernel32',use_last_error=True)
        self.api.OpenFileMappingW.argtypes=[ctypes.c_ulong,ctypes.c_bool,ctypes.c_wchar_p]
        self.api.OpenFileMappingW.restype=ctypes.c_void_p
        self.api.MapViewOfFile.argtypes=[ctypes.c_void_p,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_size_t]
        self.api.MapViewOfFile.restype=ctypes.c_void_p
        self.api.UnmapViewOfFile.argtypes=[ctypes.c_void_p]
        self.api.CloseHandle.argtypes=[ctypes.c_void_p]
        self.handle=self.api.OpenFileMappingW(4,False,name);self.address=None
        if not self.handle:raise RuntimeError('IMU history unavailable: rebuild/restart matching UE plugin: '+name)
        self.address=self.api.MapViewOfFile(self.handle,4,0,0,HEADER.size+1024*SLOT.size)
        if not self.address:
            self.close();raise ctypes.WinError(ctypes.get_last_error())
        actual=HEADER.unpack(ctypes.string_at(self.address,HEADER.size))
        if actual[:5]!=(MAGIC,1,HEADER.size,1024,SLOT.size):
            self.close();raise RuntimeError('IMU shared-memory protocol mismatch')
        self.period=round(1e9/rate);self.latency=round(latency*1e9)
        self.last_id=0;self.last_ns=0;self.target=None
        self.samples=deque();self.overruns=0
        self.buffered_sim_age_s=0.

    def _drain(self):
        latest=ctypes.c_uint64.from_address(self.address+16).value
        if not latest:return
        if latest<self.last_id:raise RuntimeError('IMU producer reset; restart collection')
        first=latest if not self.last_id else self.last_id+1
        surviving=max(1,latest-1024+1)
        if first<surviving:
            self.overruns+=surviving-first
            raise RuntimeError('IMU history overrun; reader cannot keep up')
        for frame in range(first,latest+1):
            address=self.address+HEADER.size+(frame%1024)*SLOT.size
            before=ctypes.c_int64.from_address(address).value
            if before&1:break
            record=SLOT.unpack(ctypes.string_at(address,SLOT.size))
            after=ctypes.c_int64.from_address(address).value
            if before!=after or after&1 or record[1]!=frame:break
            ns=record[2]
            if ns<=0 or ns<self.last_ns:raise RuntimeError('IMU acquisition clock reset')
            self.last_id=frame
            if ns==self.last_ns:continue
            self.last_ns=ns
            self.samples.append((ns,record[3:]))

    def read(self,now=None):
        self._drain()
        if not self.samples:return None
        if self.target is None:self.target=self.samples[0][0]
        if self.last_ns-self.target>self.latency+500_000_000:
            raise RuntimeError('IMU publisher lag exceeds bounded history latency')
        # A bounded delay covers native physics catch-up. Stamps and noisy sensor
        # values are original; samples are neither interpolated nor repeated.
        if self.last_ns-self.latency<self.target:return None
        selected=None
        while self.samples and self.samples[0][0]<=self.target:
            selected=self.samples.popleft()
        if selected is None:return None
        self.target+=self.period
        ns,values=selected
        self.buffered_sim_age_s=(self.last_ns-ns)*1e-9
        def vector(values):return dict(zip(('x_val','y_val','z_val'),values))
        return ns,dict(orientation=dict(vector(values[:3]),w_val=values[3]),
                       angular_velocity=vector(values[4:7]),linear_acceleration=vector(values[7:10]))

    def close(self):
        if self.address:self.api.UnmapViewOfFile(self.address);self.address=None
        if self.handle:self.api.CloseHandle(self.handle);self.handle=None
