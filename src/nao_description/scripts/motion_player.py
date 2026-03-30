#!/usr/bin/env python3
# ============================================================
#  nao_description/scripts/motion_player.py
#
#  Joue une motion Choregraphe sur le NAO via ROS 2 / Gazebo.
#  Usage :
#    ros2 run nao_description motion_player.py pickup
#    ros2 run nao_description motion_player.py stand
# ============================================================
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ── Joints par contrôleur ─────────────────────────────────────────────────────
LEGS = [
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch',
    'l_ankle_pitch_joint', 'LAnkleRoll',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch',
    'r_ankle_pitch_joint', 'RAnkleRoll',
]
UPPER = [
    'HeadYaw', 'HeadPitch',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll',
    'LWristYaw', 'LHand',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll',
    'RWristYaw', 'RHand',
]

# Noms Choregraphe → noms URDF réels
REMAP = {
    'LAnklePitch': 'l_ankle_pitch_joint',
    'RAnklePitch': 'r_ankle_pitch_joint',
}


def remap_names(names):
    return [REMAP.get(n, n) for n in names]


def unify_time_axis(all_times):
    tset = set()
    for tl in all_times:
        for t in tl:
            tset.add(round(float(t), 6))
    return sorted(tset)


def interp_linear(times_j, keys_j, t):
    """Interpolation linéaire → mouvement fluide sans sauts."""
    if t <= times_j[0]:  return keys_j[0]
    if t >= times_j[-1]: return keys_j[-1]
    for i in range(len(times_j) - 1):
        if times_j[i] <= t <= times_j[i + 1]:
            r = (t - times_j[i]) / (times_j[i + 1] - times_j[i])
            return keys_j[i] + r * (keys_j[i + 1] - keys_j[i])
    return keys_j[-1]


def build_trajectory(raw_names, times, keys, wanted_joints):
    names = remap_names(raw_names)
    pairs = [(jn, names.index(jn)) for jn in wanted_joints if jn in names]
    if not pairs:
        return None
    sub_names = [p[0] for p in pairs]
    sub_times = [times[p[1]] for p in pairs]
    sub_keys  = [keys[p[1]]  for p in pairs]
    t_axis    = unify_time_axis(sub_times)

    traj = JointTrajectory()
    traj.joint_names = sub_names
    for t in t_axis:
        pt = JointTrajectoryPoint()
        pt.positions = [interp_linear(sub_times[j], sub_keys[j], t)
                        for j in range(len(sub_names))]
        sec  = int(t)
        nsec = int((t - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
    return traj


class MotionPlayer(Node):

    def __init__(self):
        super().__init__('nao_motion_player')
        self._leg_ac = ActionClient(
            self, FollowJointTrajectory,
            '/locomotion_controller/follow_joint_trajectory'
        )
        self._upper_ac = ActionClient(
            self, FollowJointTrajectory,
            '/upper_body_controller/follow_joint_trajectory'
        )

    def play(self, names, times, keys):
        """Envoie jambes + haut du corps en parallèle, attend les deux."""
        leg_traj   = build_trajectory(names, times, keys, LEGS)
        upper_traj = build_trajectory(names, times, keys, UPPER)

        pending = []
        if leg_traj:
            pending.append(self._send(self._leg_ac,   leg_traj,   'locomotion'))
        if upper_traj:
            pending.append(self._send(self._upper_ac, upper_traj, 'upper_body'))

        for fut, label in pending:
            if fut is None:
                continue
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh or not gh.accepted:
                self.get_logger().error(f'{label}: trajectoire refusée !')
                continue
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            self.get_logger().info(f'{label}: terminé.')

    def _send(self, ac, traj, label):
        self.get_logger().info(f'Attente serveur {label}...')
        if not ac.wait_for_server(timeout_sec=15.0):
            self.get_logger().error(f'Serveur {label} non disponible !')
            return None, label
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.get_logger().info(
            f'Envoi {label} : {len(traj.points)} points, '
            f'durée ≈ {traj.points[-1].time_from_start.sec}s'
        )
        return ac.send_goal_async(goal), label


def main():
    motion_name = sys.argv[1] if len(sys.argv) > 1 else 'pickup'

    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, scripts_dir)

    if motion_name == 'pickup':
        from motions.Pick_up import pickup
        names, times, keys = pickup()
    elif motion_name == 'stand':
        from motions.Stand import stand
        names, times, keys = stand()
    else:
        print('Usage: motion_player.py [pickup|stand]')
        return

    rclpy.init()
    node = MotionPlayer()
    node.play(names, times, keys)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()