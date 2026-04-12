import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Rutas
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_robot_controller = get_package_share_directory('cpp_robot_controller')

    urdf_file = os.path.join(pkg_robot_controller, 'urdf', 'c_ros_bot.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # 2. Nodo: Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    # 3. Iniciar Gazebo Moderno (Cargando un mundo vacío)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items()
    )

    # 4. Nodo: Spawn Entity (Inyecta el robot)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'mi_robot_diferencial'],
        output='screen'
    )

    # 5. EL PUENTE (Bridge): Conecta Gazebo con ROS 2
    # Transfiere el /cmd_vel de ROS a Gazebo y el /odom de Gazebo a ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # 6. Nodo: Tu Secuenciador PI
    node_pi_sequencer = Node(
        package='cpp_robot_controller',
        executable='pi_straight_sequencer',
        output='screen',
        parameters=[{
            'kp': 1.0,
            'ki': 0.05,            
            'max_integral': 1.5,
            'max_speed': 1.5,
            'tolerance': 0.02      
        }]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        node_pi_sequencer
    ])