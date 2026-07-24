# state_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32
from mavros_msgs.msg import State as MavrosState
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from drone_msgs.msg import DroneStatus

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

VIO_TIMEOUT_SEC = 0.5
GPS_TIMEOUT_SEC = 2.0

# TODO: armed / flight_mode / battery ya no llegan por PX4.
# Decidir origen MAVROS (receiver_node_ardu republicando, o suscripcion
# directa a /mavros/state y /mavros/battery aqui) antes de volver a
# rellenar estos campos de DroneStatus.


class StateNode(Node):

    def __init__(self):
        super().__init__('state_node')

        self._state = DroneStatus()
        self._last_vio_time = None
        self._last_gps_time = None

        self.create_subscription(
            Odometry, '/ov_msckf/odomimu', self._cb_vio, SENSOR_QOS)
        self.create_subscription(
            NavSatFix, '/drone/gps', self._cb_gps, SENSOR_QOS)
        self.create_subscription(
            Imu, '/drone/imu', self._cb_imu, SENSOR_QOS)
        self.create_subscription(
            MavrosState, '/drone/fcu_state', self._cb_fcu_state, SENSOR_QOS)
        self.create_subscription(
            Float32, '/drone/battery_percentage', self._cb_battery, SENSOR_QOS)

        self._pub_state = self.create_publisher(
            DroneStatus, '/drone/state',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self._pub_diag = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        self.create_timer(0.1, self._publish_state)
        self.get_logger().info('state_node iniciado')

    # --- Callbacks VIO / GPS / IMU ---
    def _cb_fcu_state(self, msg: MavrosState) -> None:
        self._state.armed = msg.armed
        self._state.flight_mode = msg.mode

    def _cb_battery(self, msg: Float32) -> None:
        self._state.battery.percentage = msg.data / 100.0

    def _cb_vio(self, msg: Odometry) -> None:
        self._last_vio_time = self.get_clock().now()
        self._state.pose.header = msg.header
        self._state.pose.pose = msg.pose
        self._state.velocity.header = msg.header
        self._state.velocity.twist = msg.twist.twist

    def _cb_gps(self, msg: NavSatFix) -> None:
        if msg.status.status < 0:
            return
        self._last_gps_time = self.get_clock().now()

    def _cb_imu(self, msg: Imu) -> None:
        pass

    # --- Watchdog ---

    def _check_watchdogs(self) -> None:
        now = self.get_clock().now()

        if self._last_vio_time is None:
            self._state.vio_ok = False
        else:
            elapsed = (now - self._last_vio_time).nanoseconds * 1e-9
            vio_ok = elapsed < VIO_TIMEOUT_SEC
            if self._state.vio_ok and not vio_ok:
                self.get_logger().warn('VIO perdido')
            elif not self._state.vio_ok and vio_ok:
                self.get_logger().info('VIO recuperado')
            self._state.vio_ok = vio_ok

        if self._last_gps_time is None:
            self._state.gps_ok = False
        else:
            elapsed = (now - self._last_gps_time).nanoseconds * 1e-9
            gps_ok = elapsed < GPS_TIMEOUT_SEC
            if self._state.gps_ok and not gps_ok:
                self.get_logger().warn('GPS perdido')
            elif not self._state.gps_ok and gps_ok:
                self.get_logger().info('GPS recuperado')
            self._state.gps_ok = gps_ok

    # --- Diagnosticos ---

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now().to_msg()
        statuses = []

        vio = DiagnosticStatus()
        vio.name = 'state_node: vio_status'
        vio.hardware_id = 'openvins'
        if self._state.vio_ok:
            vio.level = DiagnosticStatus.OK
            vio.message = 'VIO activo'
        else:
            vio.level = DiagnosticStatus.ERROR
            vio.message = 'VIO perdido - pose no fiable'
        statuses.append(vio)

        gps = DiagnosticStatus()
        gps.name = 'state_node: gps_status'
        gps.hardware_id = 'mavros_gps'
        if self._state.gps_ok:
            gps.level = DiagnosticStatus.OK
            gps.message = 'GPS activo'
        else:
            gps.level = DiagnosticStatus.WARN
            gps.message = 'GPS no disponible'
        statuses.append(gps)

        arr = DiagnosticArray()
        arr.header.stamp = now
        arr.status = statuses
        self._pub_diag.publish(arr)

    # --- Timer ---

    def _publish_state(self) -> None:
        self._check_watchdogs()
        self._publish_diagnostics()
        self._state.header.stamp = self.get_clock().now().to_msg()
        self._pub_state.publish(self._state)


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