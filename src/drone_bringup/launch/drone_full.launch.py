"""
drone_full.launch.py

Lanza todo el stack de software del dron con un solo comando.

Prerrequisitos (iniciar manualmente ANTES de ejecutar este launch):
  # Terminal 1 — ArduPilot SITL + Gazebo
  cd ~/ardupilot
  sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

  # Terminal 2 (opcional) — MAVProxy para ver telemetría
  mavproxy.py --master udp:127.0.0.1:14550

Uso:
  ros2 launch drone_bringup drone_full.launch.py
  ros2 launch drone_bringup drone_full.launch.py fcu_url:=udp://:14551@127.0.0.1:14555
  ros2 launch drone_bringup drone_full.launch.py with_vio:=true

Nodos lanzados:
  1. MAVROS          — puente MAVLink ↔ ROS 2
  2. receiver_node   — republica topics MAVROS → /drone/*
  3. emitter_node    — /drone/cmd_vel → MAVROS setpoint + VIO pose
  4. state_node      — agrega estado en /drone/state
  5. pilot_node      — piloto autónomo con PID
  (6. ov_msckf)      — VIO opcional (with_vio:=true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ------------------------------------------------------------------
    # Argumentos configurables
    # ------------------------------------------------------------------
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14551@127.0.0.1:14555',
        description='URL de conexión MAVROS al FCU (SITL o hardware real)',
    )
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='URL de Ground Control Station (vacío = sin GCS)',
    )
    with_vio_arg = DeclareLaunchArgument(
        'with_vio',
        default_value='false',
        description='Lanzar OpenVINS junto con el stack principal',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulación (true para GAZEBO). Si no está Gazebo activo, no hay topic /clock',
    )

    auto_arm_arg = DeclareLaunchArgument(
        'auto_arm',
        default_value='false',
        description='Armar automáticamente al conectar (true solo en SITL)',
    )
    auto_arm = LaunchConfiguration('auto_arm')

    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    with_vio = LaunchConfiguration('with_vio')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------------------------------------------
    # Rutas de configuración
    # ------------------------------------------------------------------
    bringup_share = get_package_share_directory('drone_bringup')
    pilot_params = os.path.join(bringup_share, 'config', 'pilot_params.yaml')
    mavros_config = os.path.join(bringup_share, 'config', 'mavros_apm.yaml')

    # ------------------------------------------------------------------
    # 1. MAVROS
    # ------------------------------------------------------------------
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[
            {'fcu_url': fcu_url},
            {'gcs_url': gcs_url},
            {'target_system_id': 1},
            {'target_component_id': 1},
            {'fcu_protocol': 'v2.0'},
        ],
    )

    # ------------------------------------------------------------------
    # 2. Bridge de entrada: MAVROS → /drone/*
    # ------------------------------------------------------------------
    receiver_node = Node(
        package='drone_bridge',
        executable='receiver_node_ardu',
        name='receiver_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 3. Bridge de salida: /drone/cmd_vel → MAVROS setpoint + VIO pose
    # ------------------------------------------------------------------
    emitter_node = Node(
        package='drone_bridge',
        executable='emitter_node',
        name='emitter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'auto_arm': auto_arm,
        }],
    )

    # ------------------------------------------------------------------
    # 4. Nodo de estado: agrega pose VIO, GPS, batería → /drone/state
    # ------------------------------------------------------------------
    state_node = Node(
        package='drone_state',
        executable='state_node',
        name='state_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 5. Piloto autónomo: máquina de estados + PID
    # ------------------------------------------------------------------
    pilot_node = Node(
    package='drone_pilot',
    executable='pilot_node',
    name='pilot_node',
    output='screen',
    parameters=[
        pilot_params,
        {'use_sim_time': use_sim_time},
        {'require_vio': False},   # ← quitar cuando el VIO esté validado
    ],
    )

    # ------------------------------------------------------------------
    # 6. VIO (opcional): OpenVINS
    # ------------------------------------------------------------------
    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'vio.launch.py')
        ),
        condition=IfCondition(with_vio),
    )

    # ------------------------------------------------------------------
    # Descripción del launch
    # ------------------------------------------------------------------
    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        with_vio_arg,
        use_sim_time_arg,
        auto_arm_arg,

        LogInfo(msg='=== Iniciando stack completo del dron ==='),
        LogInfo(msg=['FCU URL: ', fcu_url]),

        mavros_node,
        receiver_node,
        emitter_node,
        state_node,
        pilot_node,
        vio_launch,

        LogInfo(msg='=== Todos los nodos lanzados. ==='),
        LogInfo(msg='El piloto espera órdenes asíncronas en /drone/pilot_cmd '
                    '(drone_msgs/msg/PilotCommand).'),
        LogInfo(msg='Ejemplo de despegue (con el dron armado en GUIDED):'),
        LogInfo(msg="  ros2 topic pub -1 /drone/pilot_cmd drone_msgs/msg/PilotCommand "
                    "\"{command_type: 0, takeoff_altitude: 5.0}\""),
        LogInfo(msg='Ejemplo de movimiento relativo al dron (5 m adelante):'),
        LogInfo(msg="  ros2 topic pub -1 /drone/pilot_cmd drone_msgs/msg/PilotCommand "
                    "\"{command_type: 1, x: 5.0, y: 0.0, z: 0.0, yaw: 0.0}\""),
        LogInfo(msg='Ejemplo de aterrizaje:'),
        LogInfo(msg="  ros2 topic pub -1 /drone/pilot_cmd drone_msgs/msg/PilotCommand "
                    "\"{command_type: 3}\""),

    ])
