#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    PythonExpression,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    load_gripper = LaunchConfiguration("load_gripper")
    declare_load_gripper_arg = DeclareLaunchArgument(
        "load_gripper",
        default_value="False",
        description="Use Franka Gripper as end-effector if True",
    )
    ## Spawn empty world in ignition-gazebo.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    
    ## Set name of controller configuration file.
    controller_config_name = PythonExpression(
        [
          "'panda_ros_controllers.yaml'"
        ]
    )
    
    # Load robot controllers file given the configuration file above.
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_panda_controller"),
            "config",
            controller_config_name,
        ]
    )
    
    # Load robot description content that will be used by 'gz_spawn_entity" node below.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("my_panda_description"),
                    "robots",
                    "panda_arm.urdf.xacro",
                ]
            ),
            "hand:=",
            load_gripper,
            "use_sim:=True",
            "simulation_controllers_config_file:=",
            robot_controllers,
        ]
    )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "panda_arm_manipulator",
            "-allow_renaming",
            "true",
            "-string",
            robot_description_content
        ],
        output="screen",
    )
    
    return LaunchDescription(
        [   declare_load_gripper_arg,
            gz_sim, ## Spawn empty world in ignition-gazebo.
            gz_spawn_entity ## Spawn Franka Emika Robot on the table.
        ]
    )
