"""
state_node.py

Agrega datos de sensores y estado del FCU en un único topic /drone/state
(DroneStatus). Específico para hardware real con ArduPilot vía MAVROS
y odometría VIO de OpenVINS.

Fuentes:
  Pose      → /ov_msckf/odomimu        (OpenVINS)
  FCU       → /mavros/state            (ArduPilot vía MAVROS)
  Batería   → /mavros/battery          (ArduPilot vía MAVROS)
  GPS       → /drone/gps               (receiver_node_ardu)

Publicaciones:
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

VIO_TIMEOUT_SEC         = 0.5
GPS_TIMEOUT_SEC         = 2.0
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

        # ── Estado interno ────────────────────────────────────────────────
        self._state          = DroneStatus()
        self._last_vio_time  = None
        self._last_gps_time  = None

        # Batería: valor inicial 100 % mientras no llegue dato real
        self._state.battery.percentage = 1.0

        # ── Suscripciones ─────────────────────────────────────────────────

        # Pose VIO de OpenVINS
        self.create_subscription(
            Odometry,
            '/ov_msckf/odomimu',
            self._cb_vio,
            SENSOR_QOS,
        )

        # Estado del FCU: armed, mode — MAVROS publica con RELIABLE
        self.create_subscription(
            MavrosState,
            '/mavros/state',
            self._cb_mavros_state,
            RELIABLE_QOS,
        )

        # Batería: puede tardar en llegar, el valor inicial 100 % aguanta
        self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self._cb_mavros_battery,
            SENSOR_QOS,
        )

        # GPS watchdog — solo actualiza el flag gps_ok
        self.create_subscription(
            NavSatFix,
            '/drone/gps',
            self._cb_gps,
            SENSOR_QOS,
        )

        # ── Publicaciones ─────────────────────────────────────────────────
        self._pub_state = self.create_publisher(
            DroneStatus, '/drone/state', RELIABLE_QOS)
        self._pub_diag = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # Timer principal: 10 Hz
        self.create_timer(0.1, self._publish_state)
        self.get_logger().info('state_node iniciado — publicando /drone/state @ 10 Hz')

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────────

    def _cb_vio(self, msg: Odometry) -> None:
        self._last_vio_time              = self.get_clock().now()
        self._state.pose.header          = msg.header
        self._state.pose.pose            = msg.pose
        self._state.velocity.header      = msg.header
        self._state.velocity.twist       = msg.twist.twist

    def _cb_gps(self, msg: NavSatFix) -> None:
        if msg.status.status >= 0:
            self._last_gps_time = self.get_clock().now()

    def _cb_mavros_state(self, msg: MavrosState) -> None:
        prev_armed = self._state.armed
        prev_mode  = self._state.flight_mode

        self._state.armed       = msg.armed
        self._state.flight_mode = msg.mode

        if msg.armed != prev_armed:
            self.get_logger().info(
                f'Dron {"ARMADO" if msg.armed else "DESARMADO"} '
                f'(modo: {msg.mode})'
            )
        if msg.mode != prev_mode:
            self.get_logger().info(f'Modo de vuelo → {msg.mode}')

    def _cb_mavros_battery(self, msg: BatteryState) -> None:
        self._state.battery = msg

    # ────────────────────────────────────────────────────────────────────────
    # Watchdog
    # ────────────────────────────────────────────────────────────────────────

    def _check_watchdogs(self) -> None:
        now = self.get_clock().now()

        # VIO
        if self._last_vio_time is None:
            new_vio_ok = False
        else:
            elapsed    = (now - self._last_vio_time).nanoseconds * 1e-9
            new_vio_ok = elapsed < VIO_TIMEOUT_SEC

        if self._state.vio_ok and not new_vio_ok:
            self.get_logger().warn('VIO perdido — pose no fiable.')
        elif not self._state.vio_ok and new_vio_ok:
            self.get_logger().info('VIO recuperado — vio_ok = True')
        self._state.vio_ok = new_vio_ok

        # GPS
        if self._last_gps_time is None:
            new_gps_ok = False
        else:
            elapsed    = (now - self._last_gps_time).nanoseconds * 1e-9
            new_gps_ok = elapsed < GPS_TIMEOUT_SEC

        if self._state.gps_ok and not new_gps_ok:
            self.get_logger().warn('GPS perdido o sin fix.')
        elif not self._state.gps_ok and new_gps_ok:
            self.get_logger().info('GPS recuperado.')
        self._state.gps_ok = new_gps_ok

    # ────────────────────────────────────────────────────────────────────────
    # Diagnósticos
    # ────────────────────────────────────────────────────────────────────────

    def _publish_diagnostics(self) -> None:
        statuses = []

        # VIO
        vio_st = DiagnosticStatus()
        vio_st.name        = 'state_node: vio'
        vio_st.hardware_id = 'openvins'
        vio_st.level       = DiagnosticStatus.OK if self._state.vio_ok \
                             else DiagnosticStatus.ERROR
        vio_st.message     = 'VIO activo' if self._state.vio_ok \
                             else 'VIO perdido — pose no fiable'
        statuses.append(vio_st)

        # GPS
        gps_st = DiagnosticStatus()
        gps_st.name        = 'state_node: gps'
        gps_st.hardware_id = 'gps'
        gps_st.level       = DiagnosticStatus.OK if self._state.gps_ok \
                             else DiagnosticStatus.WARN
        gps_st.message     = 'GPS activo' if self._state.gps_ok \
                             else 'GPS no disponible'
        statuses.append(gps_st)

        # Batería
        pct    = self._state.battery.percentage
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
        self._state.header.stamp = self.get_clock().now().to_msg()
        self._pub_state.publish(self._state)


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