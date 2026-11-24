import ctypes
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from evtol_localization.storage import EvaluationStorage


class StorageTests(unittest.TestCase):
    def test_replace_failure_keeps_live_rows_and_recovers_latest_snapshot(self):
        with tempfile.TemporaryDirectory() as folder:
            storage=EvaluationStorage(folder)
            self.assertTrue(storage.save(dict(stamp_ns=1)))
            with patch.object(Path,'replace',side_effect=PermissionError('held summary')):
                self.assertFalse(storage.save(dict(stamp_ns=2)))
            storage.append(dict(error_3d_m=.1))
            self.assertFalse(storage.status()['healthy'])
            self.assertEqual(json.loads((Path(folder)/'summary.json').read_text())['stamp_ns'],1)
            storage.next_retry=0.
            self.assertTrue(storage.save(dict(stamp_ns=3)))
            storage.close()
            self.assertEqual(len((Path(folder)/'errors.jsonl').read_text().splitlines()),1)
            self.assertEqual(json.loads((Path(folder)/'summary.json').read_text())['stamp_ns'],3)

    @unittest.skipUnless(os.name=='nt','Native Windows sharing semantics')
    def test_real_windows_file_lock_does_not_escape_and_release_recovers(self):
        with tempfile.TemporaryDirectory() as folder:
            storage=EvaluationStorage(folder);self.assertTrue(storage.save(dict(stamp_ns=1)))
            kernel=ctypes.WinDLL('kernel32',use_last_error=True)
            kernel.CreateFileW.argtypes=[ctypes.c_wchar_p,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_void_p,ctypes.c_ulong,ctypes.c_ulong,ctypes.c_void_p]
            kernel.CreateFileW.restype=ctypes.c_void_p
            kernel.CloseHandle.argtypes=[ctypes.c_void_p]
            handle=kernel.CreateFileW(str(Path(folder)/'summary.json'),0x80000000,1,None,3,0,None)
            self.assertNotEqual(handle,ctypes.c_void_p(-1).value)
            try:
                self.assertFalse(storage.save(dict(stamp_ns=2)))
                self.assertFalse(storage.status()['healthy'])
            finally:kernel.CloseHandle(handle)
            storage.next_retry=0.;self.assertTrue(storage.save(dict(stamp_ns=3)));storage.close()

    def test_unwritable_row_stream_is_counted_without_stopping_metrics(self):
        with tempfile.TemporaryDirectory() as folder:
            storage=EvaluationStorage(folder)
            with patch.object(Path,'mkdir',side_effect=PermissionError('disk unavailable')):
                storage.append(dict(error_3d_m=.2));self.assertFalse(storage.save(dict(stamp_ns=1)))
            self.assertEqual(storage.status()['dropped_rows'],1)
            storage.next_retry=storage.stream_retry=0.
            storage.append(dict(error_3d_m=.3));self.assertTrue(storage.save(dict(stamp_ns=2)));storage.close()
