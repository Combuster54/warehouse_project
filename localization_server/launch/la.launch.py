from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch.actions import GroupAction

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution


from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    package_description = "map_server"

    # args that can be set from the command line or a default will be used
    # TextSubstitution(text="0.0") W ill only evaluate that in execution time.


    map_file_arg = DeclareLaunchArgument(
        "map_file", default_value=TextSubstitution(text="launch_part.rviz")
    )

    map_file_config = LaunchConfiguration('map_file')

    # include another launch file
    # use items because you need to pass a list with a key-value structure
    # [(key1,value_x),(key2,value_y),...]

    start_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(package_description),
                'config',
                'start_rviz_with_arguments.launch.py'
            ])
        ]),
        launch_arguments={
            'map_file': map_file_config}.items()
    )


    return LaunchDescription([
        map_file_arg,
        start_rviz_launch,
    ])


    from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo


def generate_launch_description():

    package_description = "launch_tests_pkg"

    rviz_config_file_name = LaunchConfiguration('rviz_config_file_name')

    rviz_config_file_name_arg = DeclareLaunchArgument(
        'rviz_config_file_name', default_value='launch_part.rviz')

    global_path_to_rviz_file = PathJoinSubstitution([
        FindPackageShare(package_description),
        'rviz_config',
        rviz_config_file_name
    ])

    message_path = LogInfo(
        msg=global_path_to_rviz_file)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', global_path_to_rviz_file])

    # create and return the launch description object
    return LaunchDescription(
        [
            rviz_config_file_name_arg,
            rviz_node,
            message_path
        ]
    )
    