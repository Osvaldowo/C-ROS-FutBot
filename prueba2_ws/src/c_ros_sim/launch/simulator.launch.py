import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sim = get_package_share_directory('c_ros_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    urdf_path = os.path.join(pkg_sim, 'urdf', 'c_ros_bot.urdf')
    world_path = os.path.join(pkg_sim, 'worlds', 'cancha_futbol.sdf')

    # ESTO EVITA EL ERROR DE [SystemPaths.cc:425]
    # Le decimos a Gazebo que busque modelos en la carpeta 'share' de tu workspace
    gz_resource_path = os.path.dirname(pkg_sim)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'futbot_v1', '-z', '0.15'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Inyectamos la variable de entorno a Gazebo
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path
        ),
        node_robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge
    ])