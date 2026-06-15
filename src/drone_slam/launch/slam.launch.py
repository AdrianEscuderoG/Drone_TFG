"""
slam.launch.py
 
Lanza RTAB-Map en modo SLAM monocular con odometría externa de OpenVINS.
 
Soporta dos modos seleccionables con el argumento `mode`:
 
  euroc  — Test con dataset EuRoC MAV (rosbag2).
           El bag no incluye camera_info, así que este launch lo publica
           automáticamente desde config/euroc_cam0.yaml.
           Requiere `ros2 bag play <bag> --clock` en otra terminal.
 
  drone  — Producción con el dron real o SITL.
           Topics propios del proyecto: /drone/image, /drone/camera_info
           Odometría: /ov_msckf/odomimu (publicada por drone_bringup)
 
Uso:
  # Test con EuRoC (flujo completo en 3 terminales):
  ros2 launch drone_slam slam.launch.py mode:=euroc
  ros2 launch ov_msckf subscribe.launch.py config:=euroc_mav use_stereo:=false max_cameras:=1
  ros2 bag play ~/datasets/euroc/MH_01_easy/ --clock
 
  # Producción:
  ros2 launch drone_bringup drone_full.launch.py with_vio:=true
  ros2 launch drone_slam slam.launch.py mode:=drone
 
Topics de entrada (modo euroc):
  /cam0/image_raw       sensor_msgs/Image        — imagen monocroma 20 Hz
  /cam0/camera_info     sensor_msgs/CameraInfo   — generado por este launch
  /ov_msckf/odomimu     nav_msgs/Odometry        — odometría VIO ~50 Hz
 
Topics de entrada (modo drone):
  /drone/image          sensor_msgs/Image
  /drone/camera_info    sensor_msgs/CameraInfo
  /ov_msckf/odomimu     nav_msgs/Odometry
 
Topics de salida (ambos modos):
  /rtabmap/cloud_map          PointCloud2                — nube 3D del entorno
  /rtabmap/grid_map           OccupancyGrid              — mapa 2D para Nav2
  /rtabmap/mapGraph           MarkerArray                — grafo de poses
  /rtabmap/localization_pose  PoseWithCovarianceStamped  — pose corregida
"""
 
import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
    # ------------------------------------------------------------------
    # Rutas
    # ------------------------------------------------------------------
    slam_share     = get_package_share_directory('drone_slam')
    rtabmap_params = os.path.join(slam_share, 'config', 'rtabmap_params.yaml')
    euroc_calib    = os.path.join(slam_share, 'config', 'euroc_cam0.yaml')
 
    # ------------------------------------------------------------------
    # Argumentos
    # ------------------------------------------------------------------
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='euroc',
        description='"euroc" para test con dataset EuRoC, "drone" para producción.',
    )
 
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='true para rosbag/SITL, false para hardware real.',
    )
 
    viz_arg = DeclareLaunchArgument(
        'viz',
        default_value='true',
        description='Lanzar rtabmap_viz. false para ahorrar CPU.',
    )
 
    mode         = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    viz          = LaunchConfiguration('viz')
 
    # ------------------------------------------------------------------
    # Condiciones de modo
    # ------------------------------------------------------------------
    is_euroc = PythonExpression(["'", mode, "' == 'euroc'"])
    is_drone = PythonExpression(["'", mode, "' == 'drone'"])
 
    # ------------------------------------------------------------------
    # Publicador de CameraInfo — solo modo euroc
    #
    # El bag de EuRoC no incluye /cam0/camera_info. Este nodo lo genera
    # leyendo la calibración oficial de EuRoC desde euroc_cam0.yaml y
    # publicando un CameraInfo sincronizado con cada imagen recibida.
    #
    # Por qué sincronizar con la imagen y no publicar a frecuencia fija:
    #   RTAB-Map usa ApproximateTime para emparejar image + camera_info.
    #   Si los timestamps no coinciden aproximadamente, los descarta.
    #   Escuchar la imagen y copiar su timestamp garantiza el emparejamiento.
    # ------------------------------------------------------------------
    camera_info_euroc = Node(
        package='drone_slam',
        executable='camera_info_pub',
        name='camera_info_pub',
        output='screen',
        condition=IfCondition(is_euroc),
        parameters=[{
            'yaml_path':         euroc_calib,
            'camera_info_topic': '/cam0/camera_info',
            'image_topic':       '/cam0/image_raw',
            'use_sim_time':      use_sim_time,
        }],
    )
 
    # ------------------------------------------------------------------
    # Nodo RTAB-Map — modo euroc
    # ------------------------------------------------------------------
    rtabmap_euroc = Node(
    package='rtabmap_slam',
    executable='rtabmap',
    name='rtabmap',
    output='screen',
    condition=IfCondition(is_euroc),
    parameters=[
        rtabmap_params,
        {
            'use_sim_time':  use_sim_time,
            # Forzar topics como parámetros explícitos
            'subscribe_rgb':   True,
            'subscribe_depth': False,
            'subscribe_stereo': False,
            'subscribe_odom':  True,
        },
    ],
    remappings=[
        ('odom',        '/ov_msckf/odomimu'),
        ('rgb/image',   '/cam0/image_raw'),
        ('rgb/camera_info', '/cam0/camera_info'),
        ('image',       '/cam0/image_raw'),
        ('camera_info', '/cam0/camera_info'),
    ],
    arguments=['--delete_db_on_start'],
)
 
    # ------------------------------------------------------------------
    # Nodo RTAB-Map — modo drone (producción / SITL)
    # ------------------------------------------------------------------
    rtabmap_drone = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        condition=IfCondition(is_drone),
        parameters=[
            rtabmap_params,
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('odom',        '/ov_msckf/odomimu'),
            ('image',       '/drone/image'),
            ('camera_info', '/drone/camera_info'),
        ],
    )
 
    # ------------------------------------------------------------------
    # Visualizador de RTAB-Map
    # ------------------------------------------------------------------
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        condition=IfCondition(viz),
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('odom', '/ov_msckf/odomimu'),
        ],
    )
 
    # ------------------------------------------------------------------
    # TF estático: base_link → frame de cámara
    #
    # RTAB-Map necesita la TF para proyectar features al espacio 3D.
    # Euroc: valores aproximados del MAV de EuRoC, suficientes para test.
    # Drone: sustituir por los valores reales de Kalibr (Fase 2).
    # ------------------------------------------------------------------
    '''tf_base_to_cam_euroc = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam',
        condition=IfCondition(is_euroc),
        arguments=['0.05', '-0.05', '0.0',
                   '0.0',  '0.0',  '0.0',
                   'base_link', 'cam0'],
    )'''
 
    tf_base_to_cam_drone = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_to_cam',
        condition=IfCondition(is_drone),
        # TODO: sustituir por valores reales de Kalibr
        arguments=['0.0', '0.0', '0.0',
                   '0.0', '0.0', '0.0',
                   'base_link', 'camera_optical_frame'],
    )
 
    # ------------------------------------------------------------------
    # Descripción del launch
    # ------------------------------------------------------------------
    return LaunchDescription([
        mode_arg,
        use_sim_time_arg,
        viz_arg,
 
        LogInfo(msg=['=== SLAM (RTAB-Map) arrancando en modo: ', mode, ' ===']),
 
        camera_info_euroc,
        rtabmap_euroc,
        rtabmap_drone,
        rtabmap_viz,
        #tf_base_to_cam_euroc,
        tf_base_to_cam_drone,
    ])
