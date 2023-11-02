import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'path_planner_server'
    localization_package = 'localization_server'
    map_package = "map_server"
    rviz_file = os.path.join(get_package_share_directory(package_name),'rviz','pathplanning.rviz')
    use_sim_time = True
    #~~~~~~~~~~~~~~~~~~~~~~~~path config files~~~~~~~~~~~~~~~~~~~~~~~~~~~+
    controller_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'bt.yaml')
    planner_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'recovery.yaml')
    filters_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'filters.yaml')
    amcl_file = os.path.join(get_package_share_directory(localization_package),'config','amcl_config.yaml')
    map_file_path = os.path.join(get_package_share_directory(map_package),'config','warehouse_map_sim.yaml')

    remappings = [('/cmd_vel', '/robot/cmd_vel')]

    return LaunchDescription([    

        #~~~~~~~~~~~~~~~~~~provide map~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':map_file_path} 
                       ]),

        #~~~~~~~~~~~~~~~~~~amcl~~~~~~~~~~~~~~~~~~~~~~~~~~

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_file]

            ),
        #~~~~~~~~~~~~~~~~~~controller server~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings = remappings),
    
        #~~~~~~~~~~~~~~~~~~planner server~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
        #~~~~~~~~~~~~~~~~~~recoveries server~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'),
        #~~~~~~~~~~~~~~~~~~~~~bt navigator~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),
        #~~~~~~~~~~~~~~~~~~~~~~~map server~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]
            ),
        #~~~~~~~~~~~~~~~~~~~~~costmap filter~~~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml]
            ),
        #~~~~~~~~~~~~~~~~~~~~lifecycle manager~~~~~~~~~~~~~~~~~~~~~~~~
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator',
                                        'filter_mask_server',
                                        'costmap_filter_info_server',
                                        'map_server',
                                        'amcl']}]),

        #~~~~~~~~~~~~~~~~~~approaching server~~~~~~~~~~~~~~~~~~~~~~~~~
        # Node(
        #     package='attach_shelf',
        #     executable='approach_service_server_node',
        #     name='attach_shelf',
        #     output='screen',
        # ),
        #~~~~~~~~~~~~~~~~~~~~~~~~~~~rviz2~~~~~~~~~~~~~~~~~~~~~~~~~~~~~      
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        ),
    ])
