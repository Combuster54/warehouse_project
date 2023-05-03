import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    # my_param = DeclareLaunchArgument(
    # 'map_file',
    # default_value='warehouse_map_sim.yaml',
    # description='Descripción del parámetro'
    # )
    # map_file = os.path.join(get_package_share_directory('map_server'), 'config',  str(LaunchConfiguration('my_param')))

   # Definición del parámetro map_file
    map_file = LaunchConfiguration('map_file')
    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',
        description='Nombre del archivo YAML del mapa')

    # Obtener la ruta completa del archivo YAML del mapa
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    maps_folder = 'maps'
    map_file_path = PythonExpression(["'", package_path, "/", maps_folder, "/", map_file, "'"])

    # Comando para ejecutar el nodo que utiliza el archivo de mapa

    map_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename': map_file_path}]
        )
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')

    #RVIZ
    rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'localization.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
    return LaunchDescription([
        declare_map_file_cmd,
        map_server,
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        rviz_node
])