import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_c_ros_sim = get_package_share_directory('c_ros_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Nombres de tus archivos (¡Actualiza estos nombres si son diferentes!)
    urdf_file = os.path.join(pkg_c_ros_sim, 'urdf', 'c_ros_bot.urdf.xacro')
    world_file = os.path.join(pkg_c_ros_sim, 'worlds', 'cancha_futbol.sdf')

    # 1. Lanzar Gazebo con la cancha
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 2. Generar URDF para Robot 1 y aparecerlo en Gazebo
    doc_robot1 = xacro.process_file(urdf_file, mappings={'robot_name': 'robot_1'})
    robot_desc_1 = doc_robot1.toxml()

    spawn_robot1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc_1,
            '-name', 'robot_1',
            '-x', '-0.5',  # Posición X (Mitad izquierda)
            '-y', '0.0',   # Posición Y
            '-z', '0.05'   # Altura para que no choque con el piso al inicio
        ],
        output='screen'
    )

    # 3. Generar URDF para Robot 2 y aparecerlo en Gazebo
    doc_robot2 = xacro.process_file(urdf_file, mappings={'robot_name': 'robot_2'})
    robot_desc_2 = doc_robot2.toxml()

    spawn_robot2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc_2,
            '-name', 'robot_2',
            '-x', '0.5',   # Posición X (Mitad derecha)
            '-y', '0.0',
            '-z', '0.05',
            '-Y', '3.1416' # Rotado 180 grados para que mire hacia el centro (Yaw)
        ],
        output='screen'
    )

    # 4. El puente de comunicación (Bridge) ROS 2 <-> Gazebo
    # Esto es OBLIGATORIO para que el cmd_vel llegue a Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/robot_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/robot_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot1,
        spawn_robot2,
        bridge
    ])