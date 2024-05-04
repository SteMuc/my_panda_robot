#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    PythonExpression,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


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
    
    # 3 In simulation, this flag must be se to true
    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="True",
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
        description="Whether to use fake sensor commands",
    )

    controller_config_name = PythonExpression(
        [
            "'panda_ros_controllers.yaml'",
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("my_panda_controller"),
            "config",
            controller_config_name,
        ]
    )

    controller_manager_name = PythonExpression(
        [
            "'/simulation_controller_manager' if ",
            use_sim,
            " else '/controller_manager'",
        ]
    )

    # Get URDF via xacro
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
            "robot_ip:=",
            robot_ip,
            "use_sim:=",
            use_sim,
            "use_fake_hardware:=",
            use_fake_hardware,
            "fake_sensor_commands:=",
            fake_sensor_commands,
            "simulation_controllers_config_file:=",
            robot_controllers,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        # remappings=[
        #     ("/imu_sensor_node/imu", "/_imu/data_raw"),
        #     ("~/motors_cmd", "/_motors_cmd"),
        #     ("~/motors_response", "/_motors_response"),
        #     ("/rosbot_xl_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        # ],
        condition=UnlessCondition(use_sim),
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_arm_controller",
            "--controller-manager",
            controller_manager_name,
            "--controller-manager-timeout",
            "120",
        ],
    )

    # Delay start of robot_controller after joint_state_broadcaster
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    actions = [
        declare_load_gripper_arg,
        declare_robot_ip_arg,
        declare_use_sim_arg,
        declare_use_fake_hardware_arg,
        declare_fake_sensor_commands_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner
    ]

    return LaunchDescription(actions)