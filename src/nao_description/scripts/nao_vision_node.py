#!/usr/bin/env python3
# ============================================================
#  nao_description/scripts/nao_vision_node.py  — VERSION FINALE
#
#  Corrections majeures par rapport à la version précédente :
#    1. /cmd_vel remplacé par micro-trajectoires réelles
#       (NAO est bipède, pas de base différentielle)
#    2. Caméras désormais disponibles grâce à nao_gazebo.xacro
#       et aux bridges dans gazebo.launch.py
#
#  Usage :
#    ros2 run nao_description nao_vision_node.py
# ============================================================
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import cv2
import numpy as np

from sensor_msgs.msg import Image
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge

scripts_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, scripts_dir)
from motions.Pick_up import pickup
from motions.Stand    import stand


# ── Constantes de comportement ────────────────────────────────────────────────
PICKUP_DISTANCE  = 0.24   # m — déclenche le pickup
DROPOFF_DISTANCE = 0.198  # m — déclenche le dépôt

# Seuils HSV
RED_LOWER1 = np.array([0,   120,  70]); RED_UPPER1 = np.array([10,  255, 255])
RED_LOWER2 = np.array([170, 120,  70]); RED_UPPER2 = np.array([180, 255, 255])
BLUE_LOWER = np.array([100, 150,  50]); BLUE_UPPER = np.array([130, 255, 255])
MIN_AREA   = 400  # pixels

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
REMAP = {
    'LAnklePitch': 'l_ankle_pitch_joint',
    'RAnklePitch': 'r_ankle_pitch_joint',
}


# ── Utilitaires trajectoire ───────────────────────────────────────────────────

def remap_names(names):
    return [REMAP.get(n, n) for n in names]

def unify_time_axis(all_times):
    tset = set()
    for tl in all_times:
        for t in tl: tset.add(round(float(t), 6))
    return sorted(tset)

def interp_linear(times_j, keys_j, t):
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
    if not pairs: return None
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
        sec = int(t); nsec = int((t - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
    return traj

def _make_leg_traj(joint_names, waypoints, times):
    """
    Construit une JointTrajectory simple à partir de waypoints explicites.
    joint_names : liste de noms de joints
    waypoints   : liste de listes de positions (un par instant)
    times       : liste de temps en secondes
    """
    traj = JointTrajectory()
    traj.joint_names = joint_names
    for positions, t in zip(waypoints, times):
        pt = JointTrajectoryPoint()
        pt.positions = positions
        sec = int(t); nsec = int((t - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
    return traj


# ── Nœud principal ────────────────────────────────────────────────────────────

class NaoVisionNode(Node):

    def __init__(self):
        super().__init__('nao_vision_node')
        self.get_logger().info('=== NAO Vision Node démarrage ===')

        self.bridge = CvBridge()

        # État de la mission
        self.have_object      = False
        self.blue_have_object = False
        self.detected_color   = None
        self.object_distance  = None
        self.object_angle     = None
        self.motion_busy      = False

        # Pré-calcul des trajectoires depuis les données Choregraphe
        n, t, k = pickup()
        self._pickup_legs  = build_trajectory(n, t, k, LEGS)
        self._pickup_upper = build_trajectory(n, t, k, UPPER)

        n, t, k = stand()
        self._stand_legs   = build_trajectory(n, t, k, LEGS)
        self._stand_upper  = build_trajectory(n, t, k, UPPER)

        # Action clients
        self._leg_ac = ActionClient(
            self, FollowJointTrajectory,
            '/locomotion_controller/follow_joint_trajectory'
        )
        self._upper_ac = ActionClient(
            self, FollowJointTrajectory,
            '/upper_body_controller/follow_joint_trajectory'
        )

        # Subscribers caméras (topics bridgés par gazebo.launch.py)
        self.create_subscription(
            Image, '/nao/CameraTop/image_raw',    self._cb_top, 10)
        self.create_subscription(
            Image, '/nao/CameraBottom/image_raw', self._cb_bot, 10)

        # Boucle de décision à 5 Hz
        # (plus lent que 10 Hz pour laisser le temps aux micro-trajectoires)
        self.create_timer(0.2, self._loop)

        self.get_logger().info('Prêt. Début de la recherche...')

    # ── Détection couleur par caméra ──────────────────────────────────────────

    def _detect(self, ros_img):
        try:
            img = self.bridge.imgmsg_to_cv2(ros_img, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image error: {e}')
            return None, None, None

        h, w = img.shape[:2]
        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mr = (cv2.inRange(hsv, RED_LOWER1,  RED_UPPER1) |
              cv2.inRange(hsv, RED_LOWER2,  RED_UPPER2))
        mb =  cv2.inRange(hsv, BLUE_LOWER,  BLUE_UPPER)

        ar, ab = cv2.countNonZero(mr), cv2.countNonZero(mb)
        if ar < MIN_AREA and ab < MIN_AREA:
            return None, None, None

        color = 'red' if ar >= ab else 'blue'
        mask  = mr if color == 'red' else mb
        area  = ar if color == 'red' else ab

        M = cv2.moments(mask)
        if M['m00'] == 0:
            return None, None, None

        cx       = int(M['m10'] / M['m00'])
        ratio    = area / (h * w)
        distance = max(0.05, 0.6 / (ratio * 80 + 0.1))
        angle    = (cx - w / 2) / (w / 2) * 0.4   # rad, ±0.4 max
        return color, distance, angle

    def _cb_top(self, msg):
        c, d, a = self._detect(msg)
        if c:
            self.detected_color  = c
            self.object_distance = d
            self.object_angle    = a

    def _cb_bot(self, msg):
        if self.detected_color is None:
            c, d, a = self._detect(msg)
            if c:
                self.detected_color  = c
                self.object_distance = d
                self.object_angle    = a

    # ── Micro-trajectoires (remplacent /cmd_vel) ──────────────────────────────
    #
    # NAO est bipède → pas de base différentielle → /cmd_vel ne sert à rien.
    # On génère des trajectoires courtes (1-2 s) envoyées à locomotion_controller.
    #
    # Logique : posture stable de base (légère flexion genoux) + petite variation
    # des joints impliqués dans la direction souhaitée.

    # Posture stable de référence (position neutre debout, légère flexion)
    _STABLE = {
        'LHipYawPitch': 0.0,  'RHipYawPitch': 0.0,
        'LHipRoll':     0.0,  'RHipRoll':     0.0,
        'LHipPitch':   -0.05, 'RHipPitch':   -0.05,
        'LKneePitch':   0.08, 'RKneePitch':   0.08,
        'l_ankle_pitch_joint': -0.03,
        'r_ankle_pitch_joint': -0.03,
        'LAnkleRoll':   0.0,  'RAnkleRoll':   0.0,
    }

    def _stable_positions(self):
        return [self._STABLE[j] for j in LEGS]

    def _micro_step_forward(self, step_size: float = 0.04, duration: float = 1.2):
        """
        Avancer d'environ step_size mètres (~4 cm par défaut).
        Principe :
          t=0   → posture stable
          t=T/2 → flexion genoux + basculement cheville (déplace le COM)
          t=T   → redressement (le robot avance d'environ step_size)
        """
        j = LEGS
        q0   = self._stable_positions()

        # Mi-mouvement : flexion accrue + cheville en dorsiflexion
        q_mid = list(q0)
        idx = {name: i for i, name in enumerate(LEGS)}
        q_mid[idx['LKneePitch']] = 0.25
        q_mid[idx['RKneePitch']] = 0.25
        q_mid[idx['l_ankle_pitch_joint']] = -0.15
        q_mid[idx['r_ankle_pitch_joint']] = -0.15
        q_mid[idx['LHipPitch']] = -0.15
        q_mid[idx['RHipPitch']] = -0.15

        # Fin : redressement partiel (on garde un peu de flexion)
        q_end = list(q0)
        q_end[idx['LKneePitch']] = 0.10
        q_end[idx['RKneePitch']] = 0.10

        T = duration
        traj = _make_leg_traj(j, [q0, q_mid, q_end], [0.0, T/2, T])
        self._send_leg_motion(traj, 'STEP_FORWARD', T + 0.3)

    def _micro_step_backward(self, duration: float = 1.2):
        """Reculer d'environ 3-4 cm."""
        j = LEGS
        q0  = self._stable_positions()
        idx = {name: i for i, name in enumerate(LEGS)}

        q_mid = list(q0)
        q_mid[idx['LKneePitch']] = 0.20
        q_mid[idx['RKneePitch']] = 0.20
        q_mid[idx['l_ankle_pitch_joint']] = 0.05   # plantarflexion = recul
        q_mid[idx['r_ankle_pitch_joint']] = 0.05
        q_mid[idx['LHipPitch']] = 0.05
        q_mid[idx['RHipPitch']] = 0.05

        q_end = list(q0)

        T = duration
        traj = _make_leg_traj(j, [q0, q_mid, q_end], [0.0, T/2, T])
        self._send_leg_motion(traj, 'STEP_BACKWARD', T + 0.3)

    def _micro_turn(self, yaw: float = 0.08, duration: float = 1.0):
        """
        Petite rotation sur place.
        yaw > 0 → tourne à gauche, yaw < 0 → tourne à droite.
        Principe : décaler les hanches en yaw opposé pour pivoter le corps.
        """
        j = LEGS
        q0  = self._stable_positions()
        idx = {name: i for i, name in enumerate(LEGS)}

        q1 = list(q0)
        q1[idx['LHipYawPitch']] =  yaw
        q1[idx['RHipYawPitch']] = -yaw
        # Légère flexion pour stabilité pendant la rotation
        q1[idx['LKneePitch']] = 0.10
        q1[idx['RKneePitch']] = 0.10

        q_end = list(q0)   # retour posture neutre

        T = duration
        traj = _make_leg_traj(j, [q0, q1, q_end], [0.0, T * 0.6, T])
        self._send_leg_motion(traj, f'TURN(yaw={yaw:.2f})', T + 0.3)

    # ── Envoi de trajectoire ──────────────────────────────────────────────────

    def _send_leg_motion(self, traj, label, busy_duration):
        """Envoie une trajectoire jambes uniquement (micro-pas)."""
        self.motion_busy = True
        if not self._leg_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{label}: serveur locomotion indisponible')
            self.motion_busy = False
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        fut = self._leg_ac.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._on_goal_response(f, label))
        self.create_timer(busy_duration, self._motion_done)

    def _send_full_motion(self, leg_traj, upper_traj, label, busy_duration):
        """Envoie jambes + bras simultanément (pickup / stand)."""
        self.motion_busy = True
        self.get_logger().info(f'Motion complète: {label}')

        if leg_traj:
            if self._leg_ac.wait_for_server(timeout_sec=5.0):
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = leg_traj
                fut = self._leg_ac.send_goal_async(goal)
                fut.add_done_callback(
                    lambda f: self._on_goal_response(f, label + '/legs'))
            else:
                self.get_logger().error('locomotion server indisponible')

        if upper_traj:
            if self._upper_ac.wait_for_server(timeout_sec=5.0):
                goal = FollowJointTrajectory.Goal()
                goal.trajectory = upper_traj
                fut = self._upper_ac.send_goal_async(goal)
                fut.add_done_callback(
                    lambda f: self._on_goal_response(f, label + '/upper'))
            else:
                self.get_logger().error('upper_body server indisponible')

        self.create_timer(busy_duration, self._motion_done)

    def _on_goal_response(self, future, label):
        gh = future.result()
        if not gh or not gh.accepted:
            self.get_logger().warn(f'{label}: goal refusé')
        else:
            self.get_logger().info(f'{label}: goal accepté')

    def _motion_done(self):
        self.motion_busy = False
        self.get_logger().info('Motion terminée, boucle reprend.')

    # ── Boucle principale de décision ─────────────────────────────────────────

    def _loop(self):
        if self.motion_busy:
            return

        color    = self.detected_color
        distance = self.object_distance
        angle    = self.object_angle

        # Reset pour le prochain cycle
        self.detected_color  = None
        self.object_distance = None
        self.object_angle    = None

        # ── Aucun objet → rotation de recherche ─────────────────────────────
        if color is None:
            self.get_logger().debug('Recherche... rotation lente')
            self._micro_turn(yaw=-0.08)
            return

        self.get_logger().info(
            f'[{color}] dist={distance:.3f}m  angle={angle:.3f}rad  '
            f'have_obj={self.have_object}'
        )

        # ── Boîte ROUGE → ramasser ───────────────────────────────────────────
        if color == 'red' and not self.have_object:
            if distance > PICKUP_DISTANCE:
                if abs(angle) > 0.15:
                    # Se réorienter vers l'objet
                    yaw = -0.08 if angle > 0 else 0.08
                    self._micro_turn(yaw=yaw)
                else:
                    self._micro_step_forward()
            else:
                self.get_logger().info('>>> PICKUP <<<')
                self._send_full_motion(
                    self._pickup_legs, self._pickup_upper, 'PICKUP', 3.2)
                self.have_object = True

        elif color == 'red' and self.have_object:
            # On a déjà la boîte, chercher le dépôt bleu
            self._micro_turn(yaw=-0.08)

        # ── Zone BLEUE → déposer ─────────────────────────────────────────────
        elif color == 'blue' and self.have_object:
            self.blue_have_object = True
            if distance > DROPOFF_DISTANCE:
                if abs(angle) > 0.15:
                    yaw = -0.08 if angle > 0 else 0.08
                    self._micro_turn(yaw=yaw)
                else:
                    self._micro_step_forward()
            else:
                self.get_logger().info('>>> STAND / DROP <<<')
                self._send_full_motion(
                    self._stand_legs, self._stand_upper, 'STAND', 6.2)
                self.have_object      = False
                self.blue_have_object = False
                # Reculer après le dépôt (déclenché après la fin du stand)
                self.create_timer(7.0, self._post_drop)

        elif color == 'blue' and not self.have_object:
            self.get_logger().info('Zone bleue vue, mais pas de boîte encore.')
            self._micro_turn(yaw=-0.08)

    def _post_drop(self):
        """Reculer de deux pas après le dépôt."""
        self._micro_step_backward()
        self.create_timer(2.0, self._micro_step_backward)


# ── Point d'entrée ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = NaoVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()