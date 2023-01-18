from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# conditional sustitution for realtime node argument
def _realtime_command(condition):
    cmd = ['"--realtime" if "true" == "', condition, '" else ""']
    return PythonExpression(cmd)

def generate_launch_description():
    # declare arguments
    arguments = []
    arguments.append(
        DeclareLaunchArgument(
            "is_realtime",
            default_value="true",
            description="Arguement to determine whether this test is running in realtime process.",
        )
    )
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
            "is_echo_server",
            default_value="false",
            description="Arguement to determine whether this test is used as echo server.",
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
    is_realtime = LaunchConfiguration("is_realtime")
    using_fake_socket_can_bridge = LaunchConfiguration("using_fake_socket_can_bridge")
    send_id = LaunchConfiguration("send_id")
    receive_id = LaunchConfiguration("receive_id")
    is_echo_server = LaunchConfiguration("is_echo_server")
    test_period = LaunchConfiguration("test_period")
    test_length = LaunchConfiguration("test_length")
    logging_file_name = LaunchConfiguration("logging_file_name")

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
    )

    # declare node
    # node for configuring can bus
    configure_can_node = Node(
        package="nturt_can_parser",
        executable="configure_can.sh",
        output="both",
    )
    # node for sending/receiving can signals
    socket_can_bridge_node = Node(
        package="nturt_can_parser",
        executable="socket_can_bridge_node",
        output="both",
    )
    # node for testing rpi can latency
    nturt_rpi_can_latency_test_node = Node(
        package="nturt_rpi_can_latency_test",
        executable="nturt_rpi_can_latency_test_node",
        parameters=[{
            "send_id" : send_id,
            "receive_id" : receive_id,
            "is_echo_server": is_echo_server,
            "test_period": test_period,
            "test_length": test_length,
            "logging_file_name": logging_file_name,
        }],
        arguments=[
            _realtime_command(is_realtime),
        ],
        output="both",
        on_exit=Shutdown(),
    )
    # node for echoing the message back to the test node
    fake_socker_can_bridge_node = Node(
        package="nturt_rpi_can_latency_test",
        executable="fake_socket_can_bridge_node",
        parameters=[{
            "send_id" : send_id,
            "receive_id" : receive_id,
            "is_echo_server": is_echo_server,
        }],
        arguments=[
            _realtime_command(is_realtime),
        ],
        output="both",
    )

    # declare event handler
    delay_after_configure_can_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=configure_can_node,
            on_exit=[
                socket_can_receiver,
                socket_can_sender,
            ],
        )
    )

    # declare action group
    if_using_fake_socket_can_bridge = GroupAction(
        actions=[
            fake_socker_can_bridge_node,
        ],
        condition=IfCondition(using_fake_socket_can_bridge),
    )
    unless_using_fake_socket_can_bridge = GroupAction(
        actions=[
            configure_can_node,
            delay_after_configure_can_node,
        ],
        condition=UnlessCondition(using_fake_socket_can_bridge),
    )

    return LaunchDescription(
        arguments + [
            nturt_rpi_can_latency_test_node,
            if_using_fake_socket_can_bridge,
            unless_using_fake_socket_can_bridge,
        ]
    )
