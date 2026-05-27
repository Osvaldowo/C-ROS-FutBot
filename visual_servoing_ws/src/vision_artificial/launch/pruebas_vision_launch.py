from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declaramos una variable dinámica para el nombre del robot
    robot_namespace = LaunchConfiguration('robot_name')

    return LaunchDescription([
        # El argumento que podremos modificar desde la terminal
        DeclareLaunchArgument(
            'robot_name',
            default_value='futbot_1',
            description='Prefijo/Namespace del robot'
        ),

        # 2. Nodo de Calibración (HSV Tuning)
        Node(
            package='vision_artificial',
            executable='calibrator_node',
            name='calibrator',
            namespace=robot_namespace,
            output='screen'
        ),

        # 3. Nodo Detector (El que procesa Z y CLAHE)
        Node(
            package='vision_artificial',
            executable='detector_node',
            name='detector_vision',
            namespace=robot_namespace,
            output='screen',
            parameters=[
                {'focal_length': 634.66},
                {'alpha': 0.2}
            ]
        )
    ])
