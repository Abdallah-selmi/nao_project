# Rapport Projet NAO ROS 2

Date de mise a jour: 2026-03-01  
Workspace: `/home/abdallah/ros2_ws`  
Package principal: `nao_description`

## 1. Resume executif

Ce projet fournit une simulation complete du robot NAO avec:

- une description robot URDF/Xacro,
- un lancement Gazebo Sim Harmonic (SDF 1.10, moteur `dartsim`),
- une integration `ros2_control` pour commander les articulations,
- un script de marche open-loop point a point,
- un script de diagnostic rapide de la sante de la simulation.

Objectif principal: faire deplacer le robot en physique reelle (sans teleportation) via des trajectoires articulaires ROS 2.

## 2. Fonctionnalites du projet

### 2.1 Modelisation robot

- Modele principal: `urdf/nao.urdf.xacro`
- Extension simulation: `urdf/nao_gazebo.xacro`
- Materiaux et maillages:
  - `urdf/materials.xacro`
  - `meshes/visual/*`
  - `meshes/collision/*`

Le modele inclut les chaines cinematiques bras/jambes, inerties, collisions et plugin de controle pour Gazebo.

### 2.2 Simulation Gazebo Harmonic

- Monde: `worlds/nao_physics.world`
- SDF: version `1.10`
- Physique: `dartsim`
- Plugins systeme:
  - `gz-sim-physics-system`
  - `gz-sim-scene-broadcaster-system`
  - `gz-sim-contact-system`
  - `gz-sim-user-commands-system`

Le monde applique une friction sol ajustee (`mu/mu2 = 1.1`) et des parametres de contact robustes.

### 2.3 Controle ROS 2

- Configuration controleurs: `config/ros2_controllers.yaml`
- Controleurs utilises:
  - `joint_state_broadcaster`
  - `locomotion_controller` (`joint_trajectory_controller`)

Le controleur locomotion pilote 12 articulations de jambes (hips, genoux, chevilles).

### 2.4 Marche point a point (open-loop)

- Script: `scripts/walk_point_to_point.py`
- Principe:
  - calcul du cap cible,
  - phase de rotation open-loop,
  - phase d avancee open-loop,
  - retour posture neutre.

Le script publie des `JointTrajectory` sur `/locomotion_controller/joint_trajectory`.

### 2.5 Diagnostic simulation

- Script: `scripts/check_sim_health.py`
- Verifications:
  - sortie `gz stats` (RTF),
  - reception `/joint_states`,
  - position TF de `base_link` (alerte si chute probable).

## 3. Architecture logicielle

### 3.1 Sequence de lancement (gazebo.launch.py)

`launch/gazebo.launch.py` orchestre:

1. Chargement du Xacro en `robot_description` (`use_gazebo=true`).
2. Lancement du serveur Gazebo (`ros_gz_sim`).
3. Lancement GUI Gazebo conditionnel (`gui:=true/false`).
4. Bridge `/clock` via `ros_gz_bridge`.
5. Spawn du robot dans le monde `default`.
6. Chargement et activation des controleurs:
   - `joint_state_broadcaster`
   - `locomotion_controller`

Variables d environnement configurees:

- `GZ_SIM_RESOURCE_PATH`
- `GZ_SIM_RENDER_ENGINE=ogre`
- `GZ_SIM_RENDER_ENGINE_GUI=ogre`

### 3.2 Visualisation RViz (display.launch.py)

`launch/display.launch.py` lance:

- `robot_state_publisher`,
- `joint_state_publisher_gui`,
- `rviz2` avec `rviz/view.rviz`.

Usage conseille: RViz seul pour inspection kinematique, ou RViz en parallele de Gazebo sans `joint_state_publisher_gui` pour eviter les conflits de commande.

## 4. Cartographie des fichiers du package

| Fichier | Role |
|---|---|
| `CMakeLists.txt` | Installation des ressources et scripts executables |
| `package.xml` | Dependances ROS 2 (`ros_gz_sim`, `gz_ros2_control`, `ros2_controllers`, etc.) |
| `launch/gazebo.launch.py` | Lancement simulation complete + spawners controleurs |
| `launch/display.launch.py` | Lancement RViz + publication etat articulations en mode visualisation |
| `urdf/nao.urdf.xacro` | Modele robot principal |
| `urdf/nao_gazebo.xacro` | Parametres Gazebo + plugin `gz_ros2_control` + contacts pieds |
| `config/ros2_controllers.yaml` | Definition `controller_manager` et `locomotion_controller` |
| `worlds/nao_physics.world` | Monde physique Harmonic (`dartsim`, sol, friction, contact) |
| `scripts/walk_point_to_point.py` | Controle open-loop de marche point a point |
| `scripts/check_sim_health.py` | Diagnostic rapide de l etat simulation |
| `rviz/view.rviz` | Configuration RViz du projet |

## 5. Methodes principales du code

### 5.1 `walk_point_to_point.py`

Methodes importantes dans la classe `NaoPhysicalWalker`:

- `__init__(args)`: charge les parametres de marche et cree le publisher trajectoire.
- `_get_controller_state(name)`: lit l etat des controleurs via service `ListControllers`.
- `_switch_publisher_topic(topic)`: bascule de topic trajectoire si fallback necessaire.
- `_wait_for_controller_subscriber()`: attend la connexion du controleur avant d envoyer la marche.
- `_spin_for(duration)`: attente non bloquante avec `rclpy.spin_once`.
- `_build_phase_pose(left_leg_forward, step_scale, turn_scale)`: construit une posture de phase (lean, transfert lateral, swing/stance).
- `_publish_single_phase(...)`: publie une trajectoire a 2 points (mid-swing + final).
- `_publish_neutral(duration)`: envoie la posture neutre.
- `_execute_turn(delta_yaw)`: sequence open-loop de rotation.
- `_execute_forward(distance)`: sequence open-loop d avancee.
- `walk_from_to(...)`: pipeline complet A->B (attente, virage, avancee, neutralisation).

Fonctions hors classe:

- `normalize_angle(angle)`: normalisation sur `[-pi, pi]`.
- `parse_args()`: interface CLI (position depart/arrivee, timing, amplitudes).
- `main()`: point d entree du script.

### 5.2 `check_sim_health.py`

- `SimChecker.__init__()`: initialise listener TF et subscriber `/joint_states`.
- `SimChecker.check()`: execute les 3 checks (stats, joint_states, pose base_link).
- `main()`: init/shutdown ROS 2.

## 6. Flux de donnees ROS

Topics principaux:

- `/clock` (bridge Gazebo -> ROS 2)
- `/joint_states`
- `/locomotion_controller/joint_trajectory`
- `/locomotion_controller/controller_state`
- `/locomotion_controller/transition_event`

Services principaux:

- `/controller_manager/list_controllers`
- services de chargement/activation via `controller_manager spawner`

## 7. Procedure d execution recommandee

### 7.1 Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/abdallah/ros2_ws
colcon build --packages-select nao_description --symlink-install
source install/setup.bash
```

### 7.2 Lancer simulation

```bash
ros2 launch nao_description gazebo.launch.py
```

### 7.3 Verifier controleurs

```bash
ros2 control list_controllers
```

Attendu:

- `joint_state_broadcaster ... active`
- `locomotion_controller ... active`

### 7.4 Commander une marche

```bash
ros2 run nao_description walk_point_to_point.py \
  --start-x 0.0 --start-y 0.0 \
  --goal-x 0.35 --goal-y 0.0 \
  --initial-yaw 0.0 \
  --step-period 1.0 \
  --startup-wait 2.0 \
  --forward-speed-estimate 0.025 \
  --turn-rate-estimate 0.18 \
  --hip-pitch-amp 0.16 \
  --knee-lift-amp 0.22 \
  --ankle-pitch-amp 0.12 \
  --hip-roll-shift 0.05
```

### 7.5 Diagnostic rapide

```bash
ros2 run nao_description check_sim_health.py
```

## 8. Limites actuelles

- Marche open-loop: pas de feedback IMU/ZMP/CoM.
- Stabilite sensible au tuning des amplitudes et a la friction.
- Les joints de doigts/pouces avec mimic peuvent generer des warnings selon le moteur physique.
- Le deplacement robuste sur longues distances necessite encore calibration.

## 9. Pistes d amelioration

1. Ajouter une boucle fermee (orientation bassin, vitesse, stabilite lateral).
2. Ajouter un observateur simple de chute (TF + vitesse verticale + contact pieds).
3. Ajouter des profils de marche predefinis (safe, nominal, rapide).
4. Ajouter des tests automatises de non-regression sur les commandes de marche.
5. Separer proprement visualisation RViz seule et mode simulation physique complete.
