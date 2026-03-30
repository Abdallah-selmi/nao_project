# ROS 2 workspace (Humble) — Projet NAO

Ce workspace contient le package `nao_description` ROS 2 dans `src/` :

- `nao_description` (ament_cmake) : URDF + launch RViz pour afficher le robot.

## Prérequis (Ubuntu 22.04 / ROS 2 Humble)

```bash
sudo apt update
sudo apt install \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-rviz2 \
  ros-humble-nao-meshes
```

## Build

> Dans cet environnement, la commande `colcon` n’est pas sur le PATH, mais `python3 -m colcon` fonctionne.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
python3 -m colcon build --symlink-install
source install/setup.bash
```

Raccourcis :

```bash
./scripts/build.bash
source scripts/source.bash
```

## Lancer l’affichage du robot

```bash
ros2 launch nao_description display.launch.py
```

Si tu es dans un environnement “sandbox/CI” qui interdit l’écriture dans `~/.ros`, utilise :

```bash
./scripts/run_display.bash
```

## Notes

- Les meshes du NAO viennent du package système `nao_meshes` (apt: `ros-humble-nao-meshes`).
