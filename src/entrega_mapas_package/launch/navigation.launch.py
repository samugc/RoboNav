from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim_host', default_value='host.docker.internal'),
        Node(package='entrega_mapas_package', executable='coppelia_interface_node',
             parameters=[{'sim_host': LaunchConfiguration('sim_host')}], output='screen'),
        Node(package='entrega_mapas_package', executable='occupancy_mapper_node', output='screen'),
        Node(package='entrega_mapas_package', executable='bug2_controller_node', output='screen'),
        Node(package='entrega_mapas_package', executable='goal_manager_node', output='screen'),
    ])
