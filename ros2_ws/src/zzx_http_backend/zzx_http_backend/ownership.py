"""Cooperative local ownership shared by supported command producers."""
import fcntl
from pathlib import Path
import re


class CommandLease:
    def __init__(self, resource='zzxrobot'):
        if not re.fullmatch(r'[A-Za-z0-9_-]{1,64}', resource):
            raise ValueError('invalid ownership_id')
        root = Path.home() / '.local/state/zzxrobot/owners'
        root.mkdir(parents=True, exist_ok=True)
        self.file = open(root / (resource + '.lock'), 'a+b')
        try:
            fcntl.flock(self.file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.file.close()
            raise RuntimeError('command producer already owns ' + resource)

    def close(self):
        self.file.close()
