from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='futbot_1',
            description='Prefijo/Namespace del robot'
        ),
        
        # Nodo Planificador (Curvas de Bezier + LiDAR APF)
        Node(
            package='control',
            executable='ball_planner',
            namespace=robot_namespace,
            name='ball_planner_node',
            output='screen'
        ),
        
        # Nodo de Acción (Pure Pursuit)
        Node(
            package='control',
            executable='pure_pursuit_node',
            namespace=robot_namespace,
            name='pure_pursuit_node',
            output='screen'
        )
    ])