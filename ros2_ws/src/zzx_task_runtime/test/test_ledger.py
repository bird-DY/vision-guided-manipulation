"""Persistence, concurrent deduplication and process-death regressions."""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import pytest

from zzx_task_runtime.ledger import Conflict, Ledger, ReconciliationRequired
from zzx_task_runtime.move_arm import normalize


def test_twenty_concurrent_requests_dispatch_once(tmp_path):
    barrier = threading.Barrier(20)
    calls = []
    with Ledger(tmp_path / 'tasks.sqlite3') as ledger:
        def backend(payload, operation_id):
            calls.append(operation_id)
            time.sleep(.05)
            return {'state': 'SUCCEEDED', 'positions': payload['positions']}

        def submit(_):
            barrier.wait(timeout=5)
            return ledger.execute('same-task', {'positions': [1.]}, backend)

        with ThreadPoolExecutor(max_workers=20) as pool:
            results = list(pool.map(submit, range(20)))
        assert len(calls) == 1
        assert len({row['operation_id'] for row in results}) == 1
        assert ledger.get('same-task')['state'] == 'SUCCEEDED'
    with Ledger(tmp_path / 'tasks.sqlite3') as ledger:
        assert ledger.execute('same-task', {'positions': [1.]}, backend)['state'] == 'SUCCEEDED'
        assert len(calls) == 1


def test_same_id_conflicts_and_different_ids_cannot_overlap(tmp_path):
    with Ledger(tmp_path / 'tasks.sqlite3') as ledger:
        ledger.reserve('first', {'x': 1})
        with pytest.raises(Conflict):
            ledger.reserve('first', {'x': 2})
        with pytest.raises(ReconciliationRequired):
            ledger.reserve('second', {'x': 1})


def test_transport_timeout_blocks_new_actions_and_audits_reconciliation(tmp_path):
    def uncertain(*_):
        raise TimeoutError('response lost after sending')

    path = tmp_path / 'tasks.sqlite3'
    with Ledger(path) as ledger:
        row = ledger.execute('first', {}, uncertain)
        assert row['state'] == 'NEEDS_RECONCILIATION'
        with pytest.raises(ReconciliationRequired):
            ledger.reserve('second', {})
        with pytest.raises(ValueError):
            ledger.reconcile('first', 'FAILED', '')
        ledger.reconcile('first', 'FAILED', 'operator checked fake backend; no active goal')
        # Reconciliation changes knowledge, never automatically resends this ID.
        assert ledger.execute('first', {}, uncertain)['state'] == 'FAILED'
        assert ledger.get('first')['backend_result']['state'] == 'NEEDS_RECONCILIATION'
        assert ledger.get('first')['reconciliations'][0]['state'] == 'FAILED'
        assert ledger.reserve('second', {})[1]
    import sqlite3
    with sqlite3.connect(path) as db:
        assert db.execute('SELECT COUNT(*) FROM reconciliation_audit').fetchone()[0] == 1


def test_other_process_owner_is_rejected(tmp_path):
    path = tmp_path / 'tasks.sqlite3'
    with Ledger(path):
        with pytest.raises(BlockingIOError):
            Ledger(path)


def test_sigkill_between_dispatch_and_result_never_replays(tmp_path):
    path = tmp_path / 'tasks.sqlite3'
    marker = tmp_path / 'physical_dispatch.txt'
    child = Path(__file__).with_name('crash_dispatcher.py')
    process = subprocess.Popen([sys.executable, str(child), str(path), str(marker)])
    try:
        deadline = time.monotonic() + 5
        while not marker.exists() and time.monotonic() < deadline:
            assert process.poll() is None
            time.sleep(.01)
        assert marker.exists()
        os.kill(process.pid, signal.SIGKILL)
        process.wait(timeout=5)
        with Ledger(path) as ledger:
            row = ledger.execute('interrupted', {'target': 1},
                                 lambda *_: pytest.fail('must never replay'))
            assert row['state'] == 'NEEDS_RECONCILIATION'
            with pytest.raises(ReconciliationRequired):
                ledger.reserve('new-id', {'target': 1})
        assert marker.read_text() == 'one dispatch'
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=5)


def test_invalid_payload_has_no_persistent_intent(tmp_path):
    with Ledger(tmp_path / 'tasks.sqlite3') as ledger:
        with pytest.raises(ValueError):
            ledger.reserve('bad', {'x': float('nan')})
        with pytest.raises(ValueError):
            ledger.reserve('', {})
        assert ledger.get('bad') is None


def test_joint_reordering_and_numeric_normalization():
    first = dict(joint_names=['b', 'a'], positions=[2, 1], velocity_scaling=1,
                 acceleration_scaling=1, plan_only=False, timeout_seconds=5)
    second = dict(first, joint_names=['a', 'b'], positions=[1., 2.])
    assert normalize(first, '/fake') == normalize(second, '/fake')
    with pytest.raises(ValueError):
        normalize(dict(first, positions=[True, 2]), '/fake')
    with pytest.raises(ValueError):
        normalize(dict(first, unknown=1), '/fake')


def test_database_failure_before_intent_never_dispatches(tmp_path):
    # A read-only SQLite query mode simulates storage refusing the intent write.
    import sqlite3
    with Ledger(tmp_path / 'tasks.sqlite3') as ledger:
        ledger._db.execute('PRAGMA query_only=ON')
        with pytest.raises(sqlite3.OperationalError):
            ledger.execute('never-send', {}, lambda *_: pytest.fail('must not dispatch'))
