import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    rs_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tkg_catch_ball_launcher'), 'launch'), '/rs_launch.py']),
            )

    urg_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('tkg_catch_ball_launcher'), 'launch'), '/urg_node2.launch.py']),
            )

    sample_controller_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('corejp_sample_controller'),
                        'launch',
                        'controller.launch.py'
                    ])
                ]),
            )

    controller_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('tkg_autorobot_controller'),
                        'launch',
                        'controller_low_speed.launch.py'
                    ])
                ]),
            )

    catch_ball_launcher = Node(
        package="tkg_catch_ball_launcher",
        executable="launcher",
        name="launcher",
        output="log",
    )

    return LaunchDescription([
        rs_launch,
        urg_launch,
        controller_launch,
        catch_ball_launcher,
    ])

