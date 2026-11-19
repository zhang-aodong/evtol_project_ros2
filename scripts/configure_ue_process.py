"""Keep hidden Windows 11 simulation timers from being throttled."""
import argparse
import ctypes
import os


def configure(pid):
    if os.name!='nt':return
    api=ctypes.WinDLL('kernel32',use_last_error=True)
    api.OpenProcess.argtypes=[ctypes.c_uint,ctypes.c_bool,ctypes.c_uint];api.OpenProcess.restype=ctypes.c_void_p
    api.SetProcessInformation.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_uint]
    api.CloseHandle.argtypes=[ctypes.c_void_p]
    class PowerState(ctypes.Structure):
        _fields_=[('Version',ctypes.c_ulong),('ControlMask',ctypes.c_ulong),('StateMask',ctypes.c_ulong)]
    process=api.OpenProcess(0x0200|0x1000,False,pid)
    if not process:raise ctypes.WinError(ctypes.get_last_error())
    try:
        # EXECUTION_SPEED and IGNORE_TIMER_RESOLUTION, disabled per process.
        state=PowerState(1,1|4,0)
        if not api.SetProcessInformation(process,4,ctypes.byref(state),ctypes.sizeof(state)):
            raise ctypes.WinError(ctypes.get_last_error())
    finally:api.CloseHandle(process)
    print('UE Windows timer/CPU throttling disabled for process',pid)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--pid',type=int,required=True)
    configure(p.parse_args().pid)
