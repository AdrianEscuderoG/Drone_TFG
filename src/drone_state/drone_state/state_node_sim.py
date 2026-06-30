"""
state_node_sim.py

Agrega datos de sensores y estado del FCU en un único topic /drone/state
(DroneStatus). Específico para ArduPilot vía MAVROS, con dos fuentes de
odometría seleccionables.

Parámetros ROS 2
────────────────
  odometry_source   'mavros_local' → /mavros/local_position/odom  (SITL, ground truth)
                    'openvins'     → /ov_msckf/odomimu             (hardware real + VIO)
                    Default: 'mavros_local'

Publicaciones
─────────────
  /drone/state    drone_msgs/DroneStatus        @ 10 Hz
  /diagnostics    diagnostic_msgs/DiagnosticArray
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, BatteryState
from mavros_msgs.msg import State as MavrosState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from drone_msgs.msg import DroneStatus

# ── Constantes ────────────────────────────────────────────────────────────────

VIO_TIMEOUT_SEC         = 0.5   # si no llega odometría en 500 ms → vio_ok = False
GPS_TIMEOUT_SEC         = 2.0   # si no llega NavSatFix en 2 s    → gps_ok = False
BATTERY_WARN_THRESHOLD  = 0.20
BATTERY_ERROR_THRESHOLD = 0.10

# ── Perfiles QoS ──────────────────────────────────────────────────────────────

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ── Nodo ──────────────────────────────────────────────────────────────────────

class StateNode(Node):

    def __init__(self):
        super().__init__('state_node')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('odometry_source', 'mavros_local')
        self._odom_src = self.get_parameter('odometry_source').value

        self.get_logger().info(
            f'Configuración → odometry_source={self._odom_src!r} '
            'flight_controller=ardupilot'
        )

        # ── Estado interno ────────────────────────────────────────────────
        self._drone_state    = DroneStatus()
        self._last_odom_time = None
        self._last_gps_time  = None

        # Batería: valor inicial 100 % mientras no llegue dato real.
        # En SITL, /mavros/battery puede no publicar nada.
        self._drone_state.battery.percentage = 1.0

        # ── Suscripción a la fuente de odometría ──────────────────────────
        if self._odom_src == 'mavros_local':
            self.create_subscription(
                Odometry,
                '/mavros/local_position/odom',
                self._cb_odom,
                SENSOR_QOS,
            )
            self.get_logger().info(
                'Fuente de pose: /mavros/local_position/odom (ground truth SITL)'
            )
        elif self._odom_src == 'openvins':
            self.create_subscription(
                Odometry,
                '/ov_msckf/odomimu',
                self._cb_odom,
                SENSOR_QOS,
            )
            self.get_logger().info(
                'Fuente de pose: /ov_msckf/odomimu (OpenVINS VIO)'
            )
        else:
            self.get_logger().fatal(
                f'Valor desconocido para odometry_source: {self._odom_src!r}. '
                'Valores válidos: "mavros_local", "openvins".'
            )
            raise ValueError(f'odometry_source inválido: {self._odom_src!r}')

        # ── Suscripciones ArduPilot ───────────────────────────────────────
        # /mavros/state: armed, mode, connected — publicado a ~1 Hz con RELIABLE
        self.create_subscription(
            MavrosState,
            '/mavros/state',
            self._cb_mavros_state,
            RELIABLE_QOS,
        )

        # /mavros/battery: puede no existir en SITL — el valor inicial
        # 100 % se mantiene si no llega ningún mensaje.
        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._cb_mavros_battery,
            SENSOR_QOS,
        )

        # ── Suscripción GPS (watchdog) ────────────────────────────────────
        # receiver_node_ardu republica /mavros/global_position/global aquí.
        # Solo actualiza el flag gps_ok; el piloto no usa GPS directamente.
        self.create_subscription(
            NavSatFix, '/drone/gps', self._cb_gps, SENSOR_QOS)

        # ── Publicaciones ─────────────────────────────────────────────────
        self._pub_state = self.create_publisher(
            DroneStatus, '/drone/state', RELIABLE_QOS)
        self._pub_diag = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # Timer principal: 10 Hz
        self.create_timer(0.1, self._publish_state)
        self.get_logger().info(
            'state_node activo — publicando /drone/state @ 10 Hz'
        )

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks: odometría
    # ────────────────────────────────────────────────────────────────────────

    def _cb_odom(self, msg: Odometry) -> None:
        """
        Callback compartido para mavros_local y openvins.
        Ambos publican nav_msgs/Odometry con la misma estructura de campos.

        Campos mapeados:
          msg.pose        → DroneStatus.pose.pose      (PoseWithCovariance)
          msg.twist.twist → DroneStatus.velocity.twist (Twist, sin covarianza)
        """
        self._last_odom_time                 = self.get_clock().now()
        self._drone_state.pose.header        = msg.header
        self._drone_state.pose.pose          = msg.pose
        self._drone_state.velocity.header    = msg.header
        self._drone_state.velocity.twist     = msg.twist.twist

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks: GPS
    # ────────────────────────────────────────────────────────────────────────

    def _cb_gps(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:
            self._last_gps_time = self.get_clock().now()

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks: ArduPilot
    # ────────────────────────────────────────────────────────────────────────

    def _cb_mavros_state(self, msg: MavrosState) -> None:
        prev_armed = self._drone_state.armed
        prev_mode  = self._drone_state.flight_mode

        self._drone_state.armed       = msg.armed
        self._drone_state.flight_mode = msg.mode

        if msg.armed != prev_armed:
            self.get_logger().info(
                f'Dron {"ARMADO" if msg.armed else "DESARMADO"} '
                f'(modo: {msg.mode})'
            )
        if msg.mode != prev_mode:
            self.get_logger().info(f'Modo de vuelo → {msg.mode}')

    def _cb_mavros_battery(self, msg: BatteryState) -> None:
        self._drone_state.battery = msg

    # ────────────────────────────────────────────────────────────────────────
    # Watchdog
    # ────────────────────────────────────────────────────────────────────────

    def _check_watchdogs(self) -> None:
        now = self.get_clock().now()

        # Pose / VIO
        if self._last_odom_time is None:
            new_vio_ok = False
        else:
            elapsed    = (now - self._last_odom_time).nanoseconds * 1e-9
            new_vio_ok = elapsed < VIO_TIMEOUT_SEC

        if self._drone_state.vio_ok and not new_vio_ok:
            src = 'OpenVINS' if self._odom_src == 'openvins' else 'MAVROS local position'
            self.get_logger().warn(
                f'Fuente de pose perdida ({src}). '
                'Comprueba que el nodo de odometría está corriendo.'
            )
        elif not self._drone_state.vio_ok and new_vio_ok:
            self.get_logger().info('Fuente de pose recuperada — vio_ok = True')
        self._drone_state.vio_ok = new_vio_ok

        # GPS
        if self._last_gps_time is None:
            new_gps_ok = False
        else:
            elapsed    = (now - self._last_gps_time).nanoseconds * 1e-9
            new_gps_ok = elapsed < GPS_TIMEOUT_SEC

        if self._drone_state.gps_ok and not new_gps_ok:
            self.get_logger().warn('GPS perdido o sin fix.')
        elif not self._drone_state.gps_ok and new_gps_ok:
            self.get_logger().info('GPS recuperado.')
        self._drone_state.gps_ok = new_gps_ok

    # ────────────────────────────────────────────────────────────────────────
    # Diagnósticos
    # ────────────────────────────────────────────────────────────────────────

    def _publish_diagnostics(self) -> None:
        statuses = []

        # Pose
        pose_st = DiagnosticStatus()
        pose_st.name        = f'state_node: pose [{self._odom_src}]'
        pose_st.hardware_id = self._odom_src
        pose_st.level       = DiagnosticStatus.OK if self._drone_state.vio_ok \
                              else DiagnosticStatus.ERROR
        pose_st.message     = 'Pose activa' if self._drone_state.vio_ok \
                              else 'Sin odometría — pose no fiable'
        statuses.append(pose_st)

        # GPS
        gps_st = DiagnosticStatus()
        gps_st.name        = 'state_node: gps'
        gps_st.hardware_id = 'gps'
        gps_st.level       = DiagnosticStatus.OK if self._drone_state.gps_ok \
                             else DiagnosticStatus.WARN
        gps_st.message     = 'GPS activo' if self._drone_state.gps_ok \
                             else 'GPS no disponible (normal en SITL sin GPS)'
        statuses.append(gps_st)

        # Batería
        pct    = self._drone_state.battery.percentage
        bat_st = DiagnosticStatus()
        bat_st.name        = 'state_node: battery'
        bat_st.hardware_id = 'ardupilot'
        bat_st.values      = [KeyValue(key='percentage', value=f'{pct:.2f}')]
        if pct <= BATTERY_ERROR_THRESHOLD:
            bat_st.level   = DiagnosticStatus.ERROR
            bat_st.message = f'Batería crítica: {pct * 100:.0f} %'
        elif pct <= BATTERY_WARN_THRESHOLD:
            bat_st.level   = DiagnosticStatus.WARN
            bat_st.message = f'Batería baja: {pct * 100:.0f} %'
        else:
            bat_st.level   = DiagnosticStatus.OK
            bat_st.message = f'Batería OK: {pct * 100:.0f} %'
        statuses.append(bat_st)

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status       = statuses
        self._pub_diag.publish(arr)

    # ────────────────────────────────────────────────────────────────────────
    # Timer principal — 10 Hz
    # ────────────────────────────────────────────────────────────────────────

    def _publish_state(self) -> None:
        self._check_watchdogs()
        self._publish_diagnostics()
        self._drone_state.header.stamp = self.get_clock().now().to_msg()
        self._pub_state.publish(self._drone_state)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = StateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()