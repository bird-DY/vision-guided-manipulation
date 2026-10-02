"""Socket-level reproduction of the known 8087/8088 field behaviors."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import select
import signal
import subprocess
import time

import pytest
from ament_index_python.packages import get_package_prefix

from zzx_http_contracts.client import LoopbackClient, ReportedFailure, UnknownOutcome, parse_response
from zzx_http_contracts.fake import FakeServer, Scenario
from zzx_http_contracts.protocol import ARM_NAMES, joint_payload
from zzx_task_runtime.ledger import Ledger, ReconciliationRequired


def target():
    # Historical status dictionary order intentionally differs from command order.
    return dict(left_shoulder_pitch_joint=.11, left_shoulder_roll_joint=.22,
                left_elbow_roll_joint=.44, left_elbow_yaw_joint=.55,
                left_shoulder_yaw_joint=.33, left_wrist_pitch_joint=.66,
                left_wrist_yaw_joint=.77)


def test_plan_success_execute_failure_then_prepare_success():
    with FakeServer() as server:
        client = LoopbackClient(server.url)
        assert client.joints(target(), plan_only=True)['message'] == 'Planning succeeded'
        assert client.positions() == [0.] * 7
        with pytest.raises(ReportedFailure, match='Execution failed'):
            client.joints(target())
        assert server.model.execution_count == 0
        assert client.prepare()['right']['success'] is True
        posts = [(path, body) for method, path, body in server.model.requests
                 if method == 'POST'][-3:]
        assert posts == [('/api/teach_mode', {'enable': False}),
                         ('/api/control_mode', {'mode': 'pos_vel'}), ('/api/enable', {})]
        assert client.joints(target())['message'] == 'Joint motion executed'
        assert client.positions() == [.11, .22, .33, .44, .55, .66, .77]
        body = next(body for method, path, body in reversed(server.model.requests)
                    if path == '/api/joints')
        assert body['left_joints'] == [.11, .22, .33, .44, .55, .66, .77]
        assert client.request('GET', '/api/controllers')['active'] is False
        assert client.request('GET', '/api/motors')[ARM_NAMES[0]]['motor_error'] == 1


@pytest.mark.parametrize('status', [200, 400])
def test_failure_boolean_overrides_http_status(status):
    with FakeServer(scenario=Scenario(prepared=True, execution_failure=True,
                                      failure_status=status)) as server:
        with pytest.raises(ReportedFailure, match='Execution failed'):
            LoopbackClient(server.url).joints(target())
        assert server.model.execution_count == 0


@pytest.mark.parametrize('scenario', [Scenario(prepared=True, response_delay=.2),
                                     Scenario(prepared=True, disconnect=True)])
def test_missing_response_is_unknown_and_never_retried(tmp_path, scenario):
    with FakeServer(scenario=scenario) as server, Ledger(tmp_path / 'http.sqlite3') as ledger:
        client = LoopbackClient(server.url, timeout=.08)

        def dispatch(payload, operation_id):
            client.joints(payload)
            return {'state': 'SUCCEEDED'}

        result = ledger.execute('one-physical-request', target(), dispatch)
        assert result['state'] == 'NEEDS_RECONCILIATION'
        assert ledger.execute('one-physical-request', target(), dispatch)['state'] == 'NEEDS_RECONCILIATION'
        with pytest.raises(ReconciliationRequired):
            ledger.reserve('another-id', target())
        assert server.model.execution_count == 1
        # Actual fake position changed despite the client not receiving success.
        deadline = time.monotonic() + 1
        while LoopbackClient(server.url).positions() != [.11, .22, .33, .44, .55, .66, .77]:
            assert time.monotonic() < deadline
            time.sleep(.005)


def test_status_changes_and_cancel_during_motion():
    with FakeServer(scenario=Scenario(prepared=True, motion_seconds=.3)) as server:
        client = LoopbackClient(server.url)
        with ThreadPoolExecutor(max_workers=1) as pool:
            pending = pool.submit(client.joints, target())
            observer = LoopbackClient(server.url)
            deadline = time.monotonic() + 1
            while not observer.request('GET', '/api/status')['moving']:
                assert time.monotonic() < deadline
                time.sleep(.005)
            observer.command('/api/cancel', {})
            with pytest.raises(ReportedFailure, match='Canceled'):
                pending.result(timeout=2)
        assert not client.request('GET', '/api/status')['moving']
        assert client.positions() == [0.] * 7


def test_hand_sparse_update_keeps_other_nine_joints():
    with FakeServer(kind='hand') as server:
        client = LoopbackClient(server.url)
        client.hand_overrides({'index_pip': 0.})
        actual = client.request('GET', '/api/pose')['position']
        assert actual == [.5, .5, .5, .5, 0., .5, .5, .5, .5, .5]
        assert client.request('GET', '/api/status')['model'] == 'O10'
        assert server.model.execution_count == 1
        with pytest.raises(ValueError):
            client.hand_overrides({'index_pip': 1.1})
        assert server.model.execution_count == 1


@pytest.mark.parametrize('raw', [b'', b'not-json', b'[]', b'{"success":"true"}', b'{}'])
def test_malformed_success_is_not_accepted(raw):
    if raw in (b'', b'not-json', b'[]'):
        with pytest.raises(UnknownOutcome):
            parse_response(200, raw)
    else:
        client = LoopbackClient('http://127.0.0.1:1')
        client.request = lambda *_: parse_response(200, raw)
        with pytest.raises(UnknownOutcome):
            client.command('/api/joints', {})


def test_unknown_label_is_not_silently_accepted():
    client = LoopbackClient('http://127.0.0.1:1')
    client.request = lambda *_: {'other_arm': {'success': True}}
    with pytest.raises(UnknownOutcome):
        client.command('/api/enable', {})


def test_invalid_inputs_do_not_execute():
    with FakeServer(scenario=Scenario(prepared=True)) as server:
        client = LoopbackClient(server.url)
        bad = joint_payload(target())
        bad['left_joints'] = [0.] * 6
        with pytest.raises(ReportedFailure):
            client.command('/api/joints', bad)
        with pytest.raises(ValueError):
            client.joints(dict(target(), left_elbow_roll_joint=float('nan')))
        assert server.model.execution_count == 0


def test_non_loopback_destination_is_rejected_before_connecting():
    with pytest.raises(ValueError):
        LoopbackClient('http://192.168.0.22:8087')


def test_enable_success_requires_actual_left_motor_feedback():
    client = LoopbackClient('http://127.0.0.1:1')

    def response(method, path, payload=None):
        if path == '/api/control_mode':
            return {'left': {'success': True}}
        if path == '/api/enable':
            return {'right': {'success': True}}
        if path == '/api/motors':
            return {name: dict(enabled=0, fault=0, has_feedback=1) for name in ARM_NAMES}
        return {'success': True}

    client.request = response
    with pytest.raises(UnknownOutcome, match='not confirmed'):
        client.prepare()


def test_installed_server_command_starts_on_loopback():
    executable = Path(get_package_prefix('zzx_http_contracts')) / 'lib/zzx_http_contracts/competition_http_fake'
    process = subprocess.Popen([str(executable), '--kind', 'hand'],
                               stdout=subprocess.PIPE, text=True)
    try:
        assert select.select([process.stdout], [], [], 5)[0]
        info = json.loads(process.stdout.readline())
        assert info['fake'] is True
        assert LoopbackClient(info['url']).request('GET', '/api/status')['model'] == 'O10'
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)
        process.stdout.close()
