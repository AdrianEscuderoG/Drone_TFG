from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='drone_colabo',
        description='Config de OpenVINS. Usar drone_colabo cuando se calibre hardware real.'
    )

    # Bridge Gazebo → ROS 2: expone cámara e IMU del simulador como topics ROS 2
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{
            'config_file': PathJoinSubstitution([
                FindPackageShare('drone_bringup'),
                'config', 'ros_gz_bridge.yaml'
            ])
        }],
        output='screen'
    )

    # OpenVINS — consume los topics que el bridge acaba de exponer
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ov_msckf'),
                'launch', 'subscribe.launch.py'
            ])
        ]),
        launch_arguments={
            'config': LaunchConfiguration('config'),
        'use_stereo': 'false',
        'max_cameras': '1',
        }.items()
    )

    return LaunchDescription([
        config_arg,
        bridge_node,
        vio_launch,
    ])