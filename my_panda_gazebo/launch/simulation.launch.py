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
        default_value="True",
        description="Whether to use fake hardware",
    )
    # 5
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    declare_fake_sensor_commands_arg = DeclareLaunchArgument(
        "fake_sensor_commands",
        default_value="False",
        description="Whether to use fake sensor commands",
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
    
    # Load robot description content that will be used by 'gz_spawn_entity' node below.
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
            " hand:=",
            load_gripper,
            " robot_ip:=",
            robot_ip,
            " use_sim:=",
            use_sim,
            " use_fake_hardware:=",
            use_fake_hardware,
            " fake_sensor_commands:=",
            fake_sensor_commands,
            " simulation_controllers_config_file:=",
            robot_controllers,
        ]
    )
    # Spawn the Franka Arm 
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "panda_arm_manipulator",
            "-string",
            robot_description_content
        ],
        output="screen",
    )
    #
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("my_panda_bringup"),
                    "launch",
                    "bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "load_gripper": load_gripper,
            "robot_ip" :robot_ip,
            "use_sim" :use_sim,
            "use_fake_hardware" : use_fake_hardware,
            "fake_sensor_commands" : fake_sensor_commands
        }.items(),
    )
    
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    get_package_share_directory("my_panda_moveit_config"),
                    "launch",
                    "rviz.launch.py",
                ]
            )
        ),
        launch_arguments={
            "load_gripper": load_gripper,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "use_sim": use_sim,
        }.items()
    )
    
    return LaunchDescription(
        [   declare_load_gripper_arg,
            declare_robot_ip_arg,
            declare_use_sim_arg,
            declare_use_fake_hardware_arg,
            declare_fake_sensor_commands_arg,
            gz_sim, ## Spawn empty world in ignition-gazebo.
            gz_spawn_entity, ## Spawn Franka Emika Robot on the table.
            # bringup_launch,
            # moveit_rviz_launch,
            SetParameter(name="use_sim_time", value=True),
        ]
    )
