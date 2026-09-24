"""Publish diagnostics for ObjectTarget contract violations."""

import rclpy
from rclpy.node import Node

from zzx_interfaces.msg import ErrorStatus, ObjectTarget, SubsystemStatus

from .validation import ContractError, validate_object_target


class TargetContractMonitor(Node):
    """Read-only consumer proving the 0.2 target contract in a live graph."""

    def __init__(self) -> None:
        super().__init__('target_contract_monitor')
        self._publisher = self.create_publisher(
            SubsystemStatus,
            '/zzx/status/perception_contract',
            10,
        )
        self.create_subscription(
            ObjectTarget,
            '/zzx/perception/object_target',
            self._on_target,
            10,
        )

    def _on_target(self, target: ObjectTarget) -> None:
        status = SubsystemStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.name = 'perception_contract'
        status.connected = True
        status.capabilities.backend_id = 'zzx_contracts/object_target_v0_2'
        try:
            validate_object_target(target)
            status.state = SubsystemStatus.STATE_READY
            status.enabled = True
            status.message = 'target contract valid'
            status.execution.error.code = ErrorStatus.OK
        except ContractError as error:
            status.state = SubsystemStatus.STATE_DEGRADED
            status.enabled = False
            status.message = str(error)
            status.execution.error.code = ErrorStatus.INVALID_ARGUMENT
            status.execution.error.message = str(error)
        self._publisher.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetContractMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
