"""Loopback HTTP fake: a protocol state machine, not physics or a vendor server."""
import argparse
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import socket
import threading
import time

from .protocol import ARM_NAMES, HAND_NAMES, joint_payload, vector


@dataclass(frozen=True)
class Scenario:
    prepared: bool = False
    execution_failure: bool = False
    failure_status: int = 400
    response_delay: float = 0.
    disconnect: bool = False
    enable_label: str = 'right'
    motion_seconds: float = .03

    def __post_init__(self):
        if self.failure_status not in (200, 400) or self.enable_label not in ('left', 'right'):
            raise ValueError('invalid failure status or enable label')
        for duration in (self.response_delay, self.motion_seconds):
            if not math.isfinite(duration) or not 0 <= duration <= 10:
                raise ValueError('scenario durations must be in [0,10]')


class Model:
    def __init__(self, kind, scenario):
        if kind not in ('arm', 'hand'):
            raise ValueError('kind must be arm or hand')
        self.kind, self.scenario = kind, scenario
        self.lock = threading.RLock()
        self.requests = []
        self.positions = [0.] * 7 if kind == 'arm' else [.5] * 10
        self.teach = not scenario.prepared
        self.control_mode = 'pos_vel' if scenario.prepared else 'teach'
        self.enabled = scenario.prepared
        self.moving = False
        self.execution_count = 0
        self.generation = 0

    def handle(self, method, path, body):
        with self.lock:
            self.requests.append((method, path, body))
            if self.kind == 'hand':
                if method == 'GET' and path in ('/api/status', '/api/pose'):
                    return 200, dict(connected=True, hand_type='left', name='omnihand_o10',
                                     model='O10', sn='FAKE-O10', dof=10,
                                     joint_names=HAND_NAMES, position=list(self.positions)), False
                if method == 'POST' and path == '/api/set_pos':
                    self.positions = vector(body.get('position'), 10, normalized=True)
                    self.execution_count += 1
                    return 200, dict(success=True, message='Position set'), False
            else:
                if method == 'GET' and path == '/api/status':
                    # Deliberately use the historical JSON order, not command-array order.
                    order = [0, 1, 3, 4, 2, 5, 6]
                    return 200, dict(timestamp=time.time(), moving=self.moving,
                                     moveit_available=True,
                                     left_joints={ARM_NAMES[i]: self.positions[i] for i in order},
                                     left_pose=None), False
                if method == 'GET' and path == '/api/controllers':
                    return 200, dict(joint_state_available=True, active=False), False
                if method == 'GET' and path == '/api/motors':
                    return 200, {n: dict(position=self.positions[i], enabled=int(self.enabled),
                                        fault=0, has_feedback=1, feedback_age=0, motor_error=1)
                                 for i, n in enumerate(ARM_NAMES)}, False
                if method == 'POST' and path == '/api/teach_mode':
                    if type(body.get('enable')) is not bool:
                        raise ValueError('enable must be boolean')
                    self.teach = body['enable']
                    return 200, dict(success=True), False
                if method == 'POST' and path == '/api/control_mode':
                    if body.get('mode') != 'pos_vel':
                        raise ValueError('only pos_vel is modeled')
                    self.control_mode = 'pos_vel'
                    return 200, {'left': dict(success=True, message="Control mode is 'pos_vel'.")}, False
                if method == 'POST' and path == '/api/enable':
                    self.enabled = True
                    return 200, {self.scenario.enable_label: dict(success=True, message='Motors enabled')}, False
                if method == 'POST' and path == '/api/cancel':
                    self.generation += 1
                    self.moving = False
                    return 200, dict(success=True), False
                if method == 'POST' and path == '/api/joints':
                    if body.get('mode') != 'left_arm':
                        raise ValueError('only left_arm is modeled')
                    values = vector(body.get('left_joints'), 7)
                    joint_payload(dict(zip(ARM_NAMES, values)), body.get('plan_only'),
                                  body.get('velocity_scaling'), body.get('acceleration_scaling'))
                    if body['plan_only']:
                        return 200, dict(success=True, message='Planning succeeded'), False
                    if (self.teach or self.control_mode != 'pos_vel' or not self.enabled or
                            self.scenario.execution_failure or self.moving):
                        return self.scenario.failure_status, dict(success=False, message='Execution failed'), False
                    self.moving = True
                    self.execution_count += 1
                    generation = self.generation
                else:
                    return 404, dict(success=False, message='Unmodeled endpoint'), False
            if self.kind == 'hand':
                return 404, dict(success=False, message='Unmodeled endpoint'), False
        # Release the lock so status/cancel can be served during simulated motion.
        time.sleep(self.scenario.motion_seconds)
        with self.lock:
            canceled = generation != self.generation
            if not canceled:
                self.positions = values
                self.moving = False
        time.sleep(self.scenario.response_delay)
        return (400 if canceled else 200), dict(
            success=not canceled, message='Canceled' if canceled else 'Joint motion executed'), self.scenario.disconnect


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_GET(self):
        self.dispatch('GET')

    def do_POST(self):
        self.dispatch('POST')

    def dispatch(self, method):
        self.connection.settimeout(2)
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 <= length <= 65536:
                raise ValueError('invalid body size')
            body = json.loads(self.rfile.read(length)) if length else {}
            if not isinstance(body, dict):
                raise ValueError('JSON object required')
            code, result, disconnect = self.server.model.handle(method, self.path, body)
            if disconnect:
                self.connection.shutdown(socket.SHUT_RDWR)
                self.close_connection = True
                return
        except (ValueError, TypeError) as error:
            code, result = 400, dict(success=False, message=str(error))
        except OSError:
            return
        encoded = json.dumps(result, allow_nan=False).encode()
        try:
            self.send_response(code)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(encoded)))
            self.end_headers()
            self.wfile.write(encoded)
        except OSError:
            pass  # Client may time out after the simulated action has completed.


class FakeServer(ThreadingHTTPServer):
    daemon_threads = False
    block_on_close = True

    def __init__(self, kind='arm', scenario=None, port=0):
        self.model = Model(kind, scenario or Scenario())
        super().__init__(('127.0.0.1', port), Handler)

    @property
    def url(self):
        return f'http://127.0.0.1:{self.server_port}'

    def __enter__(self):
        self.thread = threading.Thread(target=self.serve_forever, kwargs={'poll_interval': .02})
        self.thread.start()
        return self

    def __exit__(self, *_):
        self.shutdown()
        self.thread.join(timeout=3)
        self.server_close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=['arm', 'hand'], default='arm')
    parser.add_argument('--port', type=int, default=0)
    parser.add_argument('--scenario', choices=['unprepared', 'prepared', 'execution_failure',
                                             'false_200', 'delayed', 'disconnect'], default='unprepared')
    args = parser.parse_args()
    scenario = Scenario(prepared=args.scenario != 'unprepared',
                        execution_failure=args.scenario in ('execution_failure', 'false_200'),
                        failure_status=200 if args.scenario == 'false_200' else 400,
                        response_delay=1. if args.scenario == 'delayed' else 0.,
                        disconnect=args.scenario == 'disconnect')
    server = FakeServer(args.kind, scenario, args.port)
    print(json.dumps(dict(fake=True, kind=args.kind, url=server.url)), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
