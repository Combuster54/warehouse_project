from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declara el argumento del archivo de mapa
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_real.yaml',
        description='Ruta al archivo de mapa YAML')

    # Crea el nodo localization_server
    localization_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'yaml_filename': LaunchConfiguration('map_file')}
                    ])

    life_cycle =         Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}])

    # Crea una descripción de lanzamiento con el nodo y el argumento
    ld = LaunchDescription()
    ld.add_action(map_file_arg)
    ld.add_action(localization_node)
    ld.add_action(life_cycle)

    return ld
