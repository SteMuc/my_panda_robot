#!/usr/bin/env python3

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Declare the launch argument for load_gripper_value
    load_gripper_value_launch_arg = DeclareLaunchArgument(
        'load_gripper_value',
        default_value='false',
        description='Use Franka Gripper as end-effector if true. Robot is loaded without '
                        'end-effector otherwise'
    )
    #  In simulation, this flag must be se to true
    declare_use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value="True",
        description="Whether simulation is used",
    )
    
    rviz_file = os.path.join(get_package_share_directory('my_panda_viz'), 'rviz',
                             'visualize_franka.rviz')

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_file],
        output='screen'
    )
    launch_franka_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(
                'my_panda_description'), 'launch/description_launch.launch.py')
        ),
        # launch_arguments={'load_gripper': 'false'}.items()
        
        # Pass the launch argument and its value
        launch_arguments={'load_gripper': LaunchConfiguration('load_gripper_value'),
                          'use_sim': LaunchConfiguration('use_sim')}.items()
    )

    return LaunchDescription(
        [   load_gripper_value_launch_arg,
            declare_use_sim_arg,
            node_joint_state_publisher_gui,
            node_rviz,
            launch_franka_description,
        ]
    )
    
    ##https://answers.ros.org/question/322874/ros2-what-is-different-between-declarelaunchargument-and-launchconfiguration/
