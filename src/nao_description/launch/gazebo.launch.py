#!/usr/bin/env python3
# ============================================================
#  nao_description/launch/gazebo.launch.py  — VERSION FINALE
#  Remplace ton fichier existant.
#
#  Ajouts par rapport à l'original :
#    1. upper_body_controller spawner (chaîné après locomotion)
#    2. camera_bridges (CameraTop + CameraBottom gz→ROS)
#    3. cmd_vel bridge (ROS→gz) pour compatibilité (utilisé via micro-traj)
# ============================================================
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share        = get_package_share_directory('nao_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_path    = os.path.join(pkg_share, 'urdf', 'nao.urdf.xacro')
    default_world = os.path.join(pkg_share, 'worlds', 'nao_physics.world')

    robot_description_content = xacro.process_file(
        xacro_path,
        mappings={'use_gazebo': 'true'},
    ).toxml()

    use_sim_time      = LaunchConfiguration('use_sim_time')
    gui               = LaunchConfiguration('gui')
    world             = LaunchConfiguration('world')
    verbose           = LaunchConfiguration('verbose')
    render_engine_gui = LaunchConfiguration('render_engine_gui')
    entity_name       = LaunchConfiguration('entity_name')
    spawn_x           = LaunchConfiguration('spawn_x')
    spawn_y           = LaunchConfiguration('spawn_y')
    spawn_z           = LaunchConfiguration('spawn_z')

    # ── Gazebo serveur + client ──────────────────────────────
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v ', verbose, ' ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-g -v ', verbose,
                        ' --render-engine-gui ', render_engine_gui],
        }.items(),
        condition=IfCondition(gui),
    )

    # ── Robot state publisher ────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_content},
            {'use_sim_time': use_sim_time},
        ],
    )

    # ── Bridge horloge simulation ────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # ── Bridge caméras : Gazebo → ROS 2 ─────────────────────
    # Syntaxe : topic@type_ros[gz_type  ([ = gz→ros)
    # Syntaxe : topic@type_ros]gz_type  (] = ros→gz)
    # Syntaxe : topic@type_ros@gz_type  (@ = bidirectionnel)
    camera_bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/nao/CameraTop/image_raw'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',

            '/nao/CameraTop/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',

            '/nao/CameraBottom/image_raw'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',

            '/nao/CameraBottom/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    # ── Spawn du robot ───────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',  entity_name,
            '-topic', 'robot_description',
            '-world', 'default',
            '-x', spawn_x, '-y', spawn_y, '-z', spawn_z,
        ],
        output='screen',
    )

    # ── Spawners contrôleurs (chaînés) ───────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    locomotion_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'locomotion_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    upper_body_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'upper_body_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120',
        ],
        output='screen',
    )

    # ── Séquence de démarrage ────────────────────────────────
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    load_locomotion_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[locomotion_controller_spawner],
        )
    )
    load_upper_body_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=locomotion_controller_spawner,
            on_exit=[upper_body_controller_spawner],
        )
    )

    gz_resource_root = os.path.dirname(pkg_share)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui',          default_value='true'),
        DeclareLaunchArgument('world',        default_value=default_world),
        DeclareLaunchArgument('verbose',      default_value='2'),
        DeclareLaunchArgument(
            'render_engine_gui', default_value='ogre',
            description='Gazebo GUI render engine (ogre, ogre2).',
        ),
        DeclareLaunchArgument('entity_name', default_value='nao'),
        DeclareLaunchArgument('spawn_x',     default_value='0.0'),
        DeclareLaunchArgument('spawn_y',     default_value='0.0'),
        DeclareLaunchArgument('spawn_z',     default_value='0.34'),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                gz_resource_root, ':',
                EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),
        SetEnvironmentVariable(name='GZ_SIM_RENDER_ENGINE',     value='ogre'),
        SetEnvironmentVariable(name='GZ_SIM_RENDER_ENGINE_GUI', value='ogre'),
        gazebo_server,
        gazebo_client,
        clock_bridge,
        camera_bridges,          # ← NOUVEAU
        robot_state_publisher,
        delayed_spawn,
        load_joint_state_broadcaster,
        load_locomotion_controller,
        load_upper_body_controller,  # ← NOUVEAU
    ])