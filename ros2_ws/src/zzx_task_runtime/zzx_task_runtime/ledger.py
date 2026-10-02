"""SQLite write-ahead intent: never automatically repeat an uncertain physical action."""
import fcntl
import hashlib
import json
from pathlib import Path
import sqlite3
import threading
import time
import uuid


class Conflict(ValueError):
    """One task ID was reused for different parameters."""


class ReconciliationRequired(RuntimeError):
    """A previous physical outcome is unknown."""


def canonical(payload):
    return json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False)


class Ledger:
    """One dispatcher owns a local database; threads may submit concurrently.

    flock fences simultaneous processes using this same database, not other
    databases or clients that bypass this gateway. Store files on local Linux FS.
    """

    def __init__(self, path):
        self.path = Path(path).expanduser().resolve()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._owner = open(str(self.path) + '.lock', 'a+b')
        self._db = None
        try:
            fcntl.flock(self._owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._db = sqlite3.connect(str(self.path), check_same_thread=False)
            self._db.row_factory = sqlite3.Row
            self._db.execute('PRAGMA journal_mode=WAL')
            self._db.execute('PRAGMA synchronous=FULL')
            self._db.executescript('''
                CREATE TABLE IF NOT EXISTS operations (
                    task_id TEXT PRIMARY KEY, payload_hash TEXT NOT NULL,
                    payload TEXT NOT NULL, state TEXT NOT NULL,
                    operation_id TEXT NOT NULL UNIQUE, backend_result TEXT,
                    updated_at REAL NOT NULL
                );
                CREATE TABLE IF NOT EXISTS reconciliation_audit (
                    id INTEGER PRIMARY KEY, task_id TEXT NOT NULL,
                    state TEXT NOT NULL, evidence TEXT NOT NULL,
                    recorded_at REAL NOT NULL
                );
            ''')
            # Exclusive process ownership proves no old dispatcher is still active.
            with self._db:
                self._db.execute('''UPDATE operations SET state=?, backend_result=?,
                    updated_at=? WHERE state='DISPATCHING' ''', (
                    'NEEDS_RECONCILIATION', canonical({'reason': 'dispatcher restarted'}),
                    time.time()))
        except BaseException:
            if self._db is not None:
                self._db.close()
            self._owner.close()
            raise

    def close(self):
        with self._lock:
            if self._db is not None:
                self._db.close()
                self._db = None
                self._owner.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    @staticmethod
    def _record(row):
        if row is None:
            return None
        result = dict(row)
        for field in ['payload', 'backend_result']:
            result[field] = json.loads(result[field]) if result[field] else None
        return result

    def get(self, task_id):
        with self._lock:
            record = self._record(self._db.execute(
                'SELECT * FROM operations WHERE task_id=?', (task_id,)).fetchone())
            if record is not None:
                record['reconciliations'] = [dict(row) for row in self._db.execute(
                    'SELECT state, evidence, recorded_at FROM reconciliation_audit '
                    'WHERE task_id=? ORDER BY id', (task_id,))]
            return record

    def reserve(self, task_id, payload):
        if not isinstance(task_id, str) or not task_id.strip() or len(task_id) > 128:
            raise ValueError('task_id must be a nonempty string of at most 128 characters')
        encoded = canonical(payload)
        digest = hashlib.sha256(encoded.encode()).hexdigest()
        with self._lock, self._db:
            current = self.get(task_id)
            if current:
                if current['payload_hash'] != digest:
                    raise Conflict('same task_id with different payload')
                return current, False
            if self._db.execute('''SELECT 1 FROM operations
                WHERE state IN ('DISPATCHING', 'NEEDS_RECONCILIATION') LIMIT 1''').fetchone():
                raise ReconciliationRequired('backend busy or previous outcome needs reconciliation')
            operation_id = uuid.uuid4().hex
            self._db.execute('INSERT INTO operations VALUES (?, ?, ?, ?, ?, ?, ?)', (
                task_id, digest, encoded, 'DISPATCHING', operation_id, None, time.time()))
        # Commit must finish before the caller can send to the backend.
        return self.get(task_id), True

    def execute(self, task_id, payload, backend):
        record, claimed = self.reserve(task_id, payload)
        if not claimed:
            return record
        try:
            outcome = backend(payload, record['operation_id'])
            if outcome['state'] not in {'SUCCEEDED', 'FAILED', 'CANCELED', 'NEEDS_RECONCILIATION'}:
                raise ValueError('invalid backend outcome')
            encoded = canonical(outcome)
        except Exception as error:
            outcome = {'state': 'NEEDS_RECONCILIATION', 'reason': repr(error)}
            encoded = canonical(outcome)
        with self._lock, self._db:
            self._db.execute('''UPDATE operations SET state=?, backend_result=?,
                updated_at=? WHERE task_id=? AND state='DISPATCHING' ''', (
                outcome['state'], encoded, time.time(), task_id))
        return self.get(task_id)

    def reconcile(self, task_id, state, evidence):
        if state not in {'SUCCEEDED', 'FAILED', 'CANCELED'}:
            raise ValueError('reconciliation requires a known terminal state')
        if not isinstance(evidence, str) or not evidence.strip():
            raise ValueError('reconciliation requires operator evidence')
        with self._lock, self._db:
            record = self.get(task_id)
            if record is None or record['state'] != 'NEEDS_RECONCILIATION':
                raise ValueError('only unknown outcomes can be reconciled')
            self._db.execute('INSERT INTO reconciliation_audit VALUES (NULL, ?, ?, ?, ?)',
                             (task_id, state, evidence, time.time()))
            # Preserve original backend evidence; the audit records the manual decision.
            self._db.execute('UPDATE operations SET state=?, updated_at=? WHERE task_id=?',
                             (state, time.time(), task_id))
        return self.get(task_id)
