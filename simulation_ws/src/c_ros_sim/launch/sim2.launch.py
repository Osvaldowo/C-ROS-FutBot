import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sim = get_package_share_directory('c_ros_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    world_path = os.path.join(pkg_sim, 'worlds', 'cancha_futbol.sdf')
    xacro_path = os.path.join(pkg_sim, 'urdf', 'c_ros_bot.urdf')

    env_var = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_sim))
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_path}'}.items()
    )

    global_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='global_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock', '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        output='screen'
    )

    launch_entities = [env_var, gazebo, global_bridge]

    # CONFIGURACIÓN DE LOS ROBOTS (1 vs 1)
    # 3.14159 (Pi) radianes son 180 grados.
    robots = [
        {'name': 'futbot_1', 'x': '-0.6', 'y': '0.0', 'Y': '0.0'},
        {'name': 'futbot_2', 'x': '0.6',  'y': '0.0', 'Y': '3.14159'}
    ]

    for bot in robots:
        prefix = f"{bot['name']}/"
        robot_desc = xacro.process_file(xacro_path, mappings={'prefix': prefix}).toxml()

        rsp = Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name=f"rsp_{bot['name']}", namespace=bot['name'], output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True, 'frame_prefix': prefix}]
        )

        spawn = Node(
            package='ros_gz_sim', executable='create', name=f"spawn_{bot['name']}",
            # Añadimos el argumento '-Y'
            arguments=['-string', robot_desc, '-name', bot['name'], '-x', bot['x'], '-y', bot['y'], '-z', '0.15', '-Y', bot['Y']],
            output='screen'
        )

        bridge = Node(
            package='ros_gz_bridge', executable='parameter_bridge', name=f"bridge_{bot['name']}",
            arguments=[
                f"/{bot['name']}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                f"/{bot['name']}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                f"/{bot['name']}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                f"/{bot['name']}/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image"
            ],
            output='screen'
        )
        launch_entities.extend([rsp, spawn, bridge])

    return LaunchDescription(launch_entities)