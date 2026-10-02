"""Strict loopback reference client for contracts, not a production HTTP driver."""
import http.client
import json
import math
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

from .protocol import ARM_NAMES, HAND_NAMES, joint_payload, vector


class ProtocolError(RuntimeError):
    pass


class ReportedFailure(ProtocolError):
    """The server reported failure; this does not prove that no movement occurred."""


class UnknownOutcome(ProtocolError):
    """A response is missing or cannot establish a known result; do not resend."""


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *_):
        return None


def parse_response(status, raw):
    try:
        data = json.loads(raw)
    except (ValueError, UnicodeError) as error:
        raise UnknownOutcome('response is not valid JSON') from error
    if not isinstance(data, dict):
        raise UnknownOutcome('response must be a JSON object')
    if status != 200 or data.get('success') is False:
        if status in (200, 400) and data.get('success') is False:
            raise ReportedFailure(f'HTTP {status}: {data.get("message", "failed")}')
        raise UnknownOutcome(f'unexpected HTTP {status}')
    return data


class LoopbackClient:
    def __init__(self, base_url, timeout=1.):
        parts = urlsplit(base_url)
        if (parts.scheme != 'http' or parts.hostname != '127.0.0.1' or
                parts.username or parts.password or parts.path not in ('', '/') or
                parts.query or parts.fragment or parts.port is None):
            raise ValueError('test client requires http://127.0.0.1:<port>')
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive')
        self.base_url, self.timeout = base_url.rstrip('/'), timeout
        self.opener = build_opener(ProxyHandler({}), NoRedirect())

    def request(self, method, path, payload=None):
        if not path.startswith('/api/') or '?' in path or '#' in path:
            raise ValueError('only API paths are supported')
        body = None if payload is None else json.dumps(payload, allow_nan=False).encode()
        request = Request(self.base_url + path, data=body, method=method,
                          headers={'Content-Type': 'application/json'})
        try:
            try:
                response = self.opener.open(request, timeout=self.timeout)
            except HTTPError as error:
                response = error
            with response:
                raw = response.read(65537)
                if len(raw) > 65536:
                    raise UnknownOutcome('response too large')
                return parse_response(response.code, raw)
        except (URLError, OSError, http.client.HTTPException) as error:
            raise UnknownOutcome('transport failed; physical outcome unknown') from error

    def command(self, path, payload):
        data = self.request('POST', path, payload)
        ack = data
        if path in ('/api/enable', '/api/control_mode'):
            labels = ('left', 'right') if path == '/api/enable' else ('left',)
            if len(data) != 1 or next(iter(data), None) not in labels:
                raise UnknownOutcome('unexpected response arm label')
            ack = next(iter(data.values()))
        if not isinstance(ack, dict) or type(ack.get('success')) is not bool:
            raise UnknownOutcome('missing boolean success acknowledgment')
        if not ack['success']:
            raise ReportedFailure(str(ack.get('message', 'operation failed')))
        return data

    def prepare(self):
        self.command('/api/teach_mode', {'enable': False})
        self.command('/api/control_mode', {'mode': 'pos_vel'})
        response = self.command('/api/enable', {})
        # Historical enable reply could say right; verify the actual left motor feedback.
        motors = self.request('GET', '/api/motors')
        for name in ARM_NAMES:
            motor = motors.get(name)
            if (not isinstance(motor, dict) or motor.get('enabled') != 1 or
                    motor.get('has_feedback') != 1 or motor.get('fault') != 0):
                raise UnknownOutcome('left motor enable state not confirmed')
        return response

    def joints(self, named, **kwargs):
        return self.command('/api/joints', joint_payload(named, **kwargs))

    def positions(self):
        data = self.request('GET', '/api/status').get('left_joints')
        if not isinstance(data, dict) or any(name not in data for name in ARM_NAMES):
            raise UnknownOutcome('left joint feedback missing')
        return vector([data[name] for name in ARM_NAMES], 7)

    def hand_overrides(self, overrides):
        if not isinstance(overrides, dict) or not overrides or not set(overrides) <= set(HAND_NAMES):
            raise ValueError('known named hand overrides required')
        positions = vector(self.request('GET', '/api/pose').get('position'), 10, normalized=True)
        for name, value in overrides.items():
            positions[HAND_NAMES.index(name)] = value
        return self.command('/api/set_pos', {'position': vector(positions, 10, normalized=True)})
