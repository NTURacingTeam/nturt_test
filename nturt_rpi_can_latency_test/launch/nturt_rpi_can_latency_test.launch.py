from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "using_fake_socket_can_bridge",
            default_value="false",
            description="Arguement to determine whether to use fake socket can bridge (only echo back).",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "send_id",
            default_value="0x010",
            description="Arguement to determine the can id that the can test message will be sent.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "receive_id",
            default_value="0x020",
            description="Arguement to determine the can id that will be received as the responding can test message.",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "test_period",
            default_value="0.1",
            description="Arguement to determine the period between each test can messages are sent [s].",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "test_length",
            default_value="60.0",
            description="Arguement to determine how long the test will run [s].",
        )
    )
    arguments.append(
        DeclareLaunchArgument(
            "logging_file_name",
            default_value=datetime.now().strftime("%Y-%m-%d-%H-%M-%S.csv"),
            description="Arguement to determine the logging file name.",
        )
    )
    
    # initialize arguments
    using_fake_socket_can_bridge = LaunchConfiguration("using_fake_socket_can_bridge")

    # declare include files
    # node for receiving can signal
    socket_can_receiver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros2_socketcan"),
                "launch",
                "socket_can_receiver.launch.py",
            ]),
        ]),
        condition=UnlessCondition(using_fake_socket_can_bridge),
    )
    # node for sending can signal
    socket_can_sender = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros2_socketcan"),
                "launch",
                "socket_can_sender.launch.py",
            ]),
        ]),
        condition=UnlessCondition(using_fake_socket_can_bridge),
    )

    includes = [
        socket_can_receiver,
        socket_can_sender,
    ]

    # declare nodes
    # node for testing rpi can latency
    nturt_rpi_can_latency_test_node = Node(
        package="nturt_rpi_can_latency_test",
        executable="nturt_rpi_can_latency_test_node",
        parameters=[{
            "send_id" : LaunchConfiguration("send_id"),
            "receive_id" : LaunchConfiguration("receive_id"),
            "test_period": LaunchConfiguration("test_period"),
            "test_length": LaunchConfiguration("test_length"),
            "logging_file_name": LaunchConfiguration("logging_file_name"),
        }],
        output="both",
        on_exit=Shutdown(),
    )
    fake_socker_can_bridge_node = Node(
        condition=IfCondition(using_fake_socket_can_bridge),
        package="nturt_rpi_can_latency_test",
        executable="fake_socket_can_bridge_node",
        parameters=[{
            "send_id" : LaunchConfiguration("send_id"),
            "receive_id" : LaunchConfiguration("receive_id"),
        }],
        output="both",
    )

    nodes = [
        nturt_rpi_can_latency_test_node,
        fake_socker_can_bridge_node
    ]

    return LaunchDescription(arguments + includes + nodes)
