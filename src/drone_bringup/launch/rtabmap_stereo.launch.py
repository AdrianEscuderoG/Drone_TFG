"""
rtabmap_stereo.launch.py

Lanza el pipeline de RTAB-Map estéreo sobre la simulación de Gazebo,
usando la odometría de MAVROS (local_position, sin OpenVINS por ahora).

Prerrequisitos (iniciar manualmente ANTES de este launch):
  # Terminal 1 — Gazebo
  gz sim -v4 -r ~/drone_colabo/src/ardupilot_gazebo/worlds/iris_stereo.sdf

  # Terminal 2 — ArduPilot SITL (después de que Gazebo esté corriendo)
  cd ~/ardupilot
  sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map --model JSON

Nodos lanzados:
  1. gz_ros_bridge              — expone /cam0, /cam1, /imu0, /clock
  2. mavros                     — odom -> base_link (tf.send=true, ver
                                   apm_config.yaml)
  3. static_transform_publisher — base_link -> cam0_link
  4. static_transform_publisher — base_link -> cam1_link
  5. stereo_camera_info_fix_node— corrige Tx de /cam1/camera_info
  6. rtabmap                    — SLAM estéreo

Uso:
  ros2 launch drone_bringup rtabmap_stereo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------
    # Argumentos configurables
    # ------------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='true siempre aquí -- este launch es solo para Gazebo',
    )

    fcu_url = LaunchConfiguration('fcu_url')
    use_sim_time = LaunchConfiguration('use_sim_time')

    bringup_share = get_package_share_directory('drone_bringup')
    bridge_config = os.path.join(bringup_share, 'config', 'ros_gz_bridge.yaml')
    mavros_config = os.path.join(bringup_share, 'config', 'apm_config.yaml')
    rtabmap_config = os.path.join(bringup_share, 'config', 'rtabmap_stereo.yaml')

    # ------------------------------------------------------------------
    #Bridge Gazebo -> ROS 2
    # ------------------------------------------------------------------
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # ------------------------------------------------------------------
    # TF estáticas base_link -> cam0_link / cam1_link
    # ------------------------------------------------------------------
    static_tf_cam0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_cam0',
        arguments=[
            '--x', '0.10', '--y', '0.06', '--z', '0.0',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '-0.5',
            '--frame-id', 'base_link', '--child-frame-id', 'cam0_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_tf_cam1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_cam1',
        arguments=[
            '--x', '0.10', '--y', '-0.06', '--z', '0.0',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '-0.5',
            '--frame-id', 'base_link', '--child-frame-id', 'cam1_link',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # Corrector de Tx en camera_info de la cámara derecha
    # ------------------------------------------------------------------
    camera_info_fix_node = Node(
        package='drone_bridge',
        executable='stereo_camera_info_fix_node',
        name='stereo_camera_info_fix_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'baseline': 0.12,
            'input_topic': '/cam1/camera_info',
            'output_topic': '/cam1/camera_info_corrected',
        }],
        output='screen',
    )

    # ------------------------------------------------------------------
    #    RTAB-Map -- SLAM estéreo
    #    Remaps: las imágenes ya llegan rectificadas de Gazebo (d=0, r=identidad),
    #    por eso image_rect apunta directo a image_raw, sin image_proc de por medio.
    # ------------------------------------------------------------------
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            rtabmap_config,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odom', '/mavros/local_position/odom'),
            ('left/image_rect', '/cam0/image_raw'),
            ('left/camera_info', '/cam0/camera_info'),
            ('right/image_rect', '/cam1/image_raw'),
            ('right/camera_info', '/cam1/camera_info_corrected'),
        ],
        arguments=['-d'],  # -d: borra la base de datos previa al arrancar
    )

    return LaunchDescription([
        use_sim_time_arg,

        LogInfo(msg='=== Iniciando pipeline RTAB-Map estéreo ==='),
        LogInfo(msg='Prerrequisito: Gazebo y SITL ya deben estar corriendo.'),

        bridge_node,
        static_tf_cam0,
        static_tf_cam1,
        camera_info_fix_node,
        rtabmap_node,

        LogInfo(msg='=== Todos los nodos lanzados. ==='),
        LogInfo(msg='Verifica con: ros2 run tf2_tools view_frames'),
    ])