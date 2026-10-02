"""Child process used to simulate death after a physical request but before a result."""
import os
from pathlib import Path
import sys
import time
from zzx_task_runtime.ledger import Ledger


def backend(*_):
    with open(sys.argv[2], 'w') as stream:
        stream.write('one dispatch')
        stream.flush()
        os.fsync(stream.fileno())
    time.sleep(60)
    return {'state': 'SUCCEEDED'}


with Ledger(Path(sys.argv[1])) as ledger:
    ledger.execute('interrupted', {'target': 1}, backend)
