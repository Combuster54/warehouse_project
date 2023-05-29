# import os
# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, PythonExpression
# from launch.actions import DeclareLaunchArgument
# import os

# def generate_launch_description():

#     # my_param = DeclareLaunchArgument(
#     # 'map_file',
#     # default_value='warehouse_map_sim.yaml',
#     # description='Descripción del parámetro'
#     # )
#     # map_file = os.path.join(get_package_share_directory('map_server'), 'config',  str(LaunchConfiguration('my_param')))

#    # Definición del parámetro map_file
#     map_file = LaunchConfiguration('map_file')
#     declare_map_file_cmd = DeclareLaunchArgument(
#         'map_file',
#         default_value='warehouse_map_sim.yaml',
#         description='Nombre del archivo YAML del mapa')

#     # Obtener la ruta completa del archivo YAML del mapa
#     package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#     maps_folder = 'maps'
#     map_file_path = PythonExpression(["'", package_path, "/", maps_folder, "/", map_file, "'"])

#     # Comando para ejecutar el nodo que utiliza el archivo de mapa

#     map_server = Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=[{'use_sim_time': True}, 
#                         {'yaml_filename': map_file_path}]
#     )
#     nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')

#     #RVIZ
#     rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localization.rviz')
#     rviz_node = Node(
#             package='rviz2',
#             executable='rviz2',
#             output='screen',
#             name='rviz_node',
#             parameters=[{'use_sim_time': True}],
#             arguments=['-d', rviz_config_dir])
#     return LaunchDescription([
#         declare_map_file_cmd,
#         map_server,
#         Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             output='screen',
#             parameters=[nav2_yaml]
#         ),
#         Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_localization',
#             output='screen',
#             parameters=[{'use_sim_time': True},
#                         {'autostart': True},
#                         {'node_names': ['map_server', 'amcl']}]
#         ),
#         rviz_node
# ])

# import os
# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, TextSubstitution

# def generate_launch_description():
    
#     nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
#     map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'warehouse_map_real.yaml')

    
#     launch_filename_arg = DeclareLaunchArgument(
#         'map_file',
#         default_value='warehouse_map_real.yaml',
#         description='A'
#     )


#     # Crear un objeto de sustituciÃ³n LaunchConfiguration para acceder al valor de launch_filename
#     launch_filename = LaunchConfiguration('map_file')
#     #RVIZ
#     rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'path.rviz')
#     rviz_node = Node(
#             package='rviz2',
#             executable='rviz2',
#             output='screen',
#             name='rviz_node',
#             parameters=[{'use_sim_time': True}],
#             arguments=['-d', rviz_config_dir])

#     # yaml_filename = '/home/user/ros2_ws/src/map_server/config/' + str(launch_filename)

#     yaml_filename = os.path.join(get_package_share_directory('map_server'), 'config', str(map_file))

#     return LaunchDescription([
#         launch_filename_arg,
#         Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=[{'use_sim_time': True}, 
#                         {'map_file': LaunchConfiguration('map_file')},
#                         {{'yaml_filename': yaml_filename}
#             }]
#         ),
#         Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             output='screen',
#             parameters=[nav2_yaml]
#         ),
#         Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_localization',
#             output='screen',
#             parameters=[{'use_sim_time': True},
#                         {'autostart': True},
#                         {'node_names': ['map_server', 'amcl']}]
#         ),
#         rviz_node
#     ])

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import LogInfo

def generate_launch_description():
    
    package_description = "map_server"

    arg_mapFile = DeclareLaunchArgument(
        'map_file', default_value= 'warehouse_map_real.yaml')

    map_config_file_name = LaunchConfiguration('map_file')

    global_path_to_map_file = PathJoinSubstitution([
        FindPackageShare(package_description),
        'config',
        map_config_file_name
    ])

    map_a =   Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': global_path_to_map_file}
            ]
        )

    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')

    amcl_node=Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        )

    #RVIZ2
    rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'path.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription([
        arg_mapFile,
        map_a,
        amcl_node,
        rviz_node,
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server','amcl']}]
        )
    ])

