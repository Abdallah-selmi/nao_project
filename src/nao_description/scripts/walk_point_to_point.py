#!/usr/bin/env python3
# ============================================================
#  nao_description/scripts/walk_point_to_point.py
#
#  Marche open-loop simple : rotation vers la cible + avancee.
#  Publie des micro-trajectoires sur locomotion_controller.
#
#  Usage :
#    ros2 run nao_description walk_point_to_point.py --start-x 0.0 --goal-x 0.3
# ============================================================
import argparse
import math
import time

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


LEG_JOINTS = [
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',
    'l_ankle_pitch_joint', 'LAnkleRoll',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch',
    'r_ankle_pitch_joint', 'RAnkleRoll',
]


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class NaoPhysicalWalker(Node):

    def __init__(self, args):
        super().__init__('nao_walk_point_to_point')
        self._args = args
        self._topic = '/locomotion_controller/joint_trajectory'
        self._pub = self.create_publisher(JointTrajectory, self._topic, 10)

        self._cm_client = self.create_client(
            ListControllers, '/controller_manager/list_controllers'
        )

        self._neutral = {
            'LHipYawPitch': 0.0,
            'LHipRoll': 0.0,
            'LHipPitch': -0.40,
            'LKneePitch': 0.70,
            'l_ankle_pitch_joint': -0.35,
            'LAnkleRoll': 0.0,
            'RHipYawPitch': 0.0,
            'RHipRoll': 0.0,
            'RHipPitch': -0.40,
            'RKneePitch': 0.70,
            'r_ankle_pitch_joint': -0.35,
            'RAnkleRoll': 0.0,
        }

        # Parametres d'amplitude
        self._hip_pitch_amp = args.hip_pitch_amp
        self._knee_lift_amp = args.knee_lift_amp
        self._ankle_pitch_amp = args.ankle_pitch_amp
        self._hip_roll_shift = args.hip_roll_shift

        # Parametres de timing
        self._step_period = args.step_period
        self._forward_speed = args.forward_speed_estimate
        self._turn_rate = args.turn_rate_estimate

        # Parametres de rotation
        self._max_yaw_per_step = 0.25
        self._yaw_amp = 0.18

    def _get_controller_state(self, name: str):
        if not self._cm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('controller_manager indisponible')
            return None
        req = ListControllers.Request()
        future = self._cm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            return None
        for ctrl in future.result().controller:
            if ctrl.name == name:
                return ctrl.state
        return None

    def _switch_publisher_topic(self, topic: str):
        if topic == self._topic:
            return
        self._topic = topic
        self._pub = self.create_publisher(JointTrajectory, self._topic, 10)
        self.get_logger().info(f'Publisher basculé sur {self._topic}')

    def _wait_for_controller_subscriber(self, timeout_sec: float = 5.0) -> bool:
        end = time.time() + timeout_sec
        while time.time() < end:
            if self._pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def _spin_for(self, duration: float):
        end = time.time() + duration
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _pose_to_list(self, pose_dict):
        return [pose_dict[j] for j in LEG_JOINTS]

    def _build_phase_pose(self, left_leg_forward: bool,
                          step_scale: float,
                          turn_scale: float):
        pose = dict(self._neutral)

        # Répartition latérale du poids
        if left_leg_forward:
            pose['LHipRoll'] = -self._hip_roll_shift
            pose['RHipRoll'] = self._hip_roll_shift
            pose['LAnkleRoll'] = self._hip_roll_shift * 0.6
            pose['RAnkleRoll'] = -self._hip_roll_shift * 0.6
        else:
            pose['LHipRoll'] = self._hip_roll_shift
            pose['RHipRoll'] = -self._hip_roll_shift
            pose['LAnkleRoll'] = -self._hip_roll_shift * 0.6
            pose['RAnkleRoll'] = self._hip_roll_shift * 0.6

        # Swing de la jambe "avant"
        if left_leg_forward:
            pose['LHipPitch'] += self._hip_pitch_amp * step_scale
            pose['LKneePitch'] += self._knee_lift_amp * step_scale
            pose['l_ankle_pitch_joint'] -= self._ankle_pitch_amp * step_scale
        else:
            pose['RHipPitch'] += self._hip_pitch_amp * step_scale
            pose['RKneePitch'] += self._knee_lift_amp * step_scale
            pose['r_ankle_pitch_joint'] -= self._ankle_pitch_amp * step_scale

        # Rotation (légère torsion des hanches)
        yaw = self._yaw_amp * max(-1.0, min(1.0, turn_scale))
        pose['LHipYawPitch'] = yaw
        pose['RHipYawPitch'] = -yaw

        return pose

    def _publish_traj(self, positions_list, t1: float, t2: float):
        traj = JointTrajectory()
        traj.joint_names = LEG_JOINTS

        pt1 = JointTrajectoryPoint()
        pt1.positions = positions_list
        pt1.time_from_start = Duration(
            sec=int(t1), nanosec=int((t1 - int(t1)) * 1e9)
        )

        pt2 = JointTrajectoryPoint()
        pt2.positions = self._pose_to_list(self._neutral)
        pt2.time_from_start = Duration(
            sec=int(t2), nanosec=int((t2 - int(t2)) * 1e9)
        )

        traj.points.append(pt1)
        traj.points.append(pt2)
        self._pub.publish(traj)

    def _publish_single_phase(self, left_leg_forward: bool,
                              step_scale: float,
                              turn_scale: float):
        mid = self._build_phase_pose(left_leg_forward, step_scale, turn_scale)
        self._publish_traj(self._pose_to_list(mid),
                           self._step_period * 0.5,
                           self._step_period)

    def _publish_neutral(self, duration: float):
        traj = JointTrajectory()
        traj.joint_names = LEG_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = self._pose_to_list(self._neutral)
        pt.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration - int(duration)) * 1e9)
        )
        traj.points.append(pt)
        self._pub.publish(traj)

    def _execute_turn(self, delta_yaw: float):
        if abs(delta_yaw) < 1e-3:
            return

        if self._turn_rate <= 0.0:
            self.get_logger().warn('turn_rate_estimate invalide, rotation ignorée')
            return

        turn_duration = abs(delta_yaw) / self._turn_rate
        steps = max(1, int(math.ceil(turn_duration / self._step_period)))
        per_step_yaw = delta_yaw / steps
        turn_scale = per_step_yaw / self._max_yaw_per_step

        self.get_logger().info(
            f'Rotation: {delta_yaw:.2f} rad en {steps} pas'
        )
        left = True
        for _ in range(steps):
            self._publish_single_phase(left, 0.6, turn_scale)
            self._spin_for(self._step_period + 0.05)
            left = not left

    def _execute_forward(self, distance: float):
        if distance <= 0.0:
            return
        if self._forward_speed <= 0.0:
            self.get_logger().warn('forward_speed_estimate invalide, avance ignorée')
            return

        forward_duration = distance / self._forward_speed
        steps = max(1, int(math.ceil(forward_duration / self._step_period)))
        self.get_logger().info(
            f'Avancée: {distance:.2f} m en {steps} pas'
        )
        left = True
        for _ in range(steps):
            self._publish_single_phase(left, 1.0, 0.0)
            self._spin_for(self._step_period + 0.05)
            left = not left

    def walk_from_to(self, start_x, start_y, goal_x, goal_y, initial_yaw):
        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx) if distance > 1e-6 else initial_yaw
        delta_yaw = normalize_angle(target_yaw - initial_yaw)

        state = self._get_controller_state('locomotion_controller')
        if state and state != 'active':
            self.get_logger().warn(
                f'locomotion_controller state={state} (attendu: active)'
            )

        if not self._wait_for_controller_subscriber(5.0):
            self.get_logger().warn(
                'Aucun subscriber locomotion détecté. '
                'Assurez-vous que les contrôleurs sont actifs.'
            )

        if self._args.startup_wait > 0.0:
            self._publish_neutral(self._args.startup_wait)
            self._spin_for(self._args.startup_wait)

        self.get_logger().info(
            f'Objectif: dx={dx:.2f}, dy={dy:.2f}, '
            f'dist={distance:.2f}, delta_yaw={delta_yaw:.2f}'
        )
        self._execute_turn(delta_yaw)
        self._execute_forward(distance)
        self._publish_neutral(1.0)
        self._spin_for(1.0)


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--start-x', type=float, default=0.0)
    p.add_argument('--start-y', type=float, default=0.0)
    p.add_argument('--goal-x', type=float, default=0.3)
    p.add_argument('--goal-y', type=float, default=0.0)
    p.add_argument('--initial-yaw', type=float, default=0.0)
    p.add_argument('--step-period', type=float, default=1.0)
    p.add_argument('--startup-wait', type=float, default=2.0)
    p.add_argument('--forward-speed-estimate', type=float, default=0.025)
    p.add_argument('--turn-rate-estimate', type=float, default=0.18)
    p.add_argument('--hip-pitch-amp', type=float, default=0.16)
    p.add_argument('--knee-lift-amp', type=float, default=0.22)
    p.add_argument('--ankle-pitch-amp', type=float, default=0.12)
    p.add_argument('--hip-roll-shift', type=float, default=0.05)
    return p.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = NaoPhysicalWalker(args)
    try:
        node.walk_from_to(
            args.start_x, args.start_y, args.goal_x, args.goal_y, args.initial_yaw
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
