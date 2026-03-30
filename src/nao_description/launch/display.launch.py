#!/usr/bin/env python3
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('nao_description')
    xacro_path = os.path.join(pkg_share, 'urdf', 'nao.urdf.xacro')

    robot_description_content = xacro.process_file(
        xacro_path,
        mappings={'use_gazebo': 'false'},
    ).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # GUI to control joint sliders
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # RViz2 with config (Fixed Frame: base_link, RobotModel)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'view.rviz')]
        )
    ])
