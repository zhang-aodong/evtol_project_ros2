"""Best-effort audit storage; filesystem failures never control the aircraft."""
import json
import os
from pathlib import Path
import time
import uuid


class EvaluationStorage:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.temp = self.folder / ('summary.' + str(os.getpid()) + '.' + uuid.uuid4().hex[:8] + '.tmp')
        self.file = None
        self.failures = 0
        self.dropped_rows = 0
        self.error = None
        self.next_retry = 0.
        self.stream_retry = 0.
        self.last_saved_ns = None

    def _failed(self, exc, stream=False):
        self.failures += 1
        self.error = str(exc)
        self.next_retry = time.monotonic() + min(5., .5 * 2 ** min(self.failures - 1, 4))
        if stream:self.stream_retry = self.next_retry
        if stream and self.file is not None:
            try:self.file.close()
            except OSError:pass
            self.file = None

    def append(self, row):
        if time.monotonic() < self.stream_retry:
            self.dropped_rows += 1
            return
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
            if self.file is None:self.file = (self.folder / 'errors.jsonl').open('a', encoding='utf8')
            self.file.write(json.dumps(row, allow_nan=False) + '\n')
        except OSError as exc:
            self.dropped_rows += 1
            self._failed(exc, stream=True)

    def save(self, report):
        if time.monotonic() < self.next_retry:return False
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
            self.temp.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf8')
            if self.file is not None:self.file.flush()
            self.temp.replace(self.folder / 'summary.json')
        except OSError as exc:
            self._failed(exc)
            return False
        self.error = None
        self.failures = 0
        self.next_retry = 0.
        self.last_saved_ns = report['stamp_ns']
        return True

    def status(self):
        return dict(healthy=self.error is None, error=self.error, consecutive_failures=self.failures,
                    dropped_rows=self.dropped_rows, last_saved_ns=self.last_saved_ns)

    def close(self):
        if self.file is not None:
            try:self.file.close()
            except OSError:pass
            self.file = None
        try:self.temp.unlink(missing_ok=True)
        except OSError:pass
