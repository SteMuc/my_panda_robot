#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1
    load_gripper = LaunchConfiguration("load_gripper")
    declare_load_gripper_arg = DeclareLaunchArgument(
        "load_gripper",
        default_value="False",
        description="Use Franka Gripper as end-effector if True",
    )
    # 2
    robot_ip = LaunchConfiguration("robot_ip")
    declare_robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="dont-care",
        description=(
            "Set the robot_ip to 'dont-care' for using the simulation."
        ),
    )
    # 3
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )
    
    # 4
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    declare_use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="False",
        description="Wheter to use fake hardware",
    )
    # 5
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    declare_fake_sensor_commands_arg = DeclareLaunchArgument(
        "fake_sensor_commands",
        default_value="False",
        description="Wheter to use fake sensor commands",
    )
    
    # 6: Real Robot flag to launch or not Franka bringup (franka.launch.py)
    real_robot = LaunchConfiguration("real_robot")
    declare_real_robot_arg = DeclareLaunchArgument(
        "real_robot",
        default_value="False",
        description="Whether to use real robot. If true, launch the FCI franka interface.",
    )
    
    ## Include franka.launch.py
    
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("my_panda_bringup"),
                    "launch",
                    "franka.launch.py"
                ]
            )
        ),
        launch_arguments={
            "load_gripper": load_gripper,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands
        }.items(),
        condition=UnlessCondition(use_fake_hardware),
    )

    # controller_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [
    #                 get_package_share_directory("rosbot_xl_manipulation_controller"),
    #                 "launch",
    #                 "controller.launch.py",
    #             ]
    #         )
    #     ),
    #     launch_arguments={
    #         "manipulator_usb_port": manipulator_usb_port,
    #         "manipulator_baud_rate": manipulator_baud_rate,
    #         "joint1_limit_min": joint1_limit_min,
    #         "joint1_limit_max": joint1_limit_max,
    #         "antenna_rotation_angle": antenna_rotation_angle,
    #         "mecanum": mecanum,
    #         "use_sim": use_sim,
    #     }.items(),
    # )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("my_panda_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
        launch_arguments={
            "load_gripper": load_gripper,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "use_sim": use_sim,
        }.items(),
    )
   
    actions = [
        declare_real_robot_arg,
        declare_load_gripper_arg,
        declare_robot_ip_arg,
        declare_use_sim_arg,
        declare_use_fake_hardware_arg,
        declare_fake_sensor_commands_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        # controller_launch,
        moveit_launch, ## Launch the move group node.
        franka_launch ## Launch Franka Bringup if using real robot (FCI).
    ]

    return LaunchDescription(actions)