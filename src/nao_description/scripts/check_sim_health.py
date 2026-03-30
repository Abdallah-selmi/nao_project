#!/usr/bin/env python3
# ============================================================
#  nao_description/scripts/check_sim_health.py
#
#  Diagnostic rapide de l'état simulation :
#   - Horloge /clock qui avance
#   - /joint_states reçoit des données
#   - TF base_link disponible
#
#  Usage :
#    ros2 run nao_description check_sim_health.py
# ============================================================
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


class SimChecker(Node):

    def __init__(self):
        super().__init__('nao_sim_checker')
        self._joint_state = None
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _on_joint_state(self, msg: JointState):
        self._joint_state = msg

    def _wait_for_clock(self, timeout_sec: float) -> bool:
        end = time.time() + timeout_sec
        last = self.get_clock().now().nanoseconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self.get_clock().now().nanoseconds
            if now != last:
                return True
            last = now
        return False

    def _wait_for_joint_states(self, timeout_sec: float) -> bool:
        end = time.time() + timeout_sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._joint_state and self._joint_state.name:
                return True
        return False

    def _lookup_base_link(self, timeout_sec: float):
        frames = [
            ('world', 'base_link'),
            ('map', 'base_link'),
            ('odom', 'base_link'),
        ]
        end = time.time() + timeout_sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            for target, source in frames:
                if self._tf_buffer.can_transform(
                    target, source, Time(), timeout_sec=0.1
                ):
                    return target, source, self._tf_buffer.lookup_transform(
                        target, source, Time()
                    )
        return None

    def check(self) -> bool:
        ok_clock = self._wait_for_clock(2.0)
        self.get_logger().info(
            f'Clock update: {"OK" if ok_clock else "FAIL"}'
        )

        ok_js = self._wait_for_joint_states(2.0)
        self.get_logger().info(
            f'Joint states: {"OK" if ok_js else "FAIL"}'
        )

        tf_result = self._lookup_base_link(2.0)
        if tf_result:
            target, source, tr = tf_result
            t = tr.transform.translation
            self.get_logger().info(
                f'TF {target}->{source}: OK '
                f'(x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f})'
            )
            ok_tf = True
        else:
            self.get_logger().info('TF base_link: FAIL')
            ok_tf = False

        return ok_clock and ok_js and ok_tf


def main():
    rclpy.init()
    node = SimChecker()
    try:
        ok = node.check()
        node.get_logger().info(
            'Simulation health: OK' if ok else 'Simulation health: DEGRADED'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
