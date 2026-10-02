"""Manual entry point for durable joint operations and evidence-based reconciliation."""
import argparse
import json
import sqlite3

from .ledger import Ledger
from .move_arm import RosMoveArmBackend, normalize


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--db', default='~/.local/state/zzxrobot/operations.sqlite3')
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run')
    run.add_argument('--task-id', required=True)
    run.add_argument('--request', required=True, help='joint request JSON file')
    run.add_argument('--action', default='/b08_fake/zzx/manipulation/move_arm')
    run.add_argument('--wait-seconds', type=float, default=10.)
    show = sub.add_parser('show')
    show.add_argument('--task-id', required=True)
    reconcile = sub.add_parser('reconcile')
    reconcile.add_argument('--task-id', required=True)
    reconcile.add_argument('--state', choices=['SUCCEEDED', 'FAILED', 'CANCELED'], required=True)
    reconcile.add_argument('--evidence', required=True)
    args = parser.parse_args()
    try:
        with Ledger(args.db) as ledger:
            if args.command == 'run':
                with open(args.request, encoding='utf-8') as stream:
                    payload = normalize(json.load(stream), args.action)
                import rclpy
                rclpy.init()
                backend = None
                try:
                    backend = RosMoveArmBackend(args.wait_seconds)
                    result = ledger.execute(args.task_id, payload, backend)
                finally:
                    if backend:
                        backend.close()
                    rclpy.shutdown()
            elif args.command == 'show':
                result = ledger.get(args.task_id)
            else:
                result = ledger.reconcile(args.task_id, args.state, args.evidence)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        if result is None or result['state'] != 'SUCCEEDED':
            raise SystemExit(2)
    except sqlite3.Error as error:
        parser.exit(2, 'Database failure; physical outcome may be unknown. '
                    'Do not resend with a new task ID: ' + str(error) + '\n')
    except (ValueError, RuntimeError, OSError) as error:
        parser.exit(2, str(error) + '\n')


if __name__ == '__main__':
    main()
