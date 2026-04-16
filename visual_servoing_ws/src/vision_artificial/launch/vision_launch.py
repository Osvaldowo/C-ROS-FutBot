import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Nodo de la Cámara (Driver)
        Node(
            package='segmentation', # Asegúrate de que este sea el nombre de tu paquete
            executable='camera_node',
            name='camera_driver',
            output='screen',
            parameters=[{'fps': 30}]
        ),

        # 2. Nodo de Calibración (HSV Tuning)
        Node(
            package='segmentation',
            executable='calibrator_node',
            name='calibrator',
            output='screen'
        ),

        # 3. Nodo Detector (El que procesa Z y CLAHE)
        Node(
            package='segmentation',
            executable='detector_node',
            name='detector_vision',
            output='screen',
            parameters=[
                {'focal_length': 634.66},
                {'alpha': 0.2}
            ]
        )
    ])
