import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from px4_msgs.msg import VehicleStatus, BatteryStatus
from drone_msgs.msg import DroneStatus

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

VIO_TIMEOUT_SEC = 0.5
GPS_TIMEOUT_SEC = 2.0

BATTERY_WARN_THRESHOLD = 0.20
BATTERY_ERROR_THRESHOLD = 0.10

NAV_STATE_NAMES = {
    0:  'MANUAL',
    1:  'ALTCTL',
    2:  'POSCTL',
    3:  'AUTO_MISSION',
    4:  'AUTO_LOITER',
    5:  'AUTO_RTL',
    10: 'ACRO',
    12: 'DESCEND',
    13: 'TERMINATION',
    14: 'OFFBOARD',
    15: 'STABILIZED',
    17: 'AUTO_TAKEOFF',
    18: 'AUTO_LAND',
    21: 'ORBIT',
}


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
            VehicleStatus, '/fmu/out/vehicle_status',
            self._cb_vehicle_status, PX4_QOS)
        self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status',
            self._cb_battery_status, PX4_QOS)

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

    # --- Callbacks PX4 ---

    def _cb_vehicle_status(self, msg: VehicleStatus) -> None:
        self._state.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self._state.flight_mode = NAV_STATE_NAMES.get(
            msg.nav_state, f'UNKNOWN_{msg.nav_state}')

    def _cb_battery_status(self, msg: BatteryStatus) -> None:
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage = msg.voltage_filtered_v
        battery.current = msg.current_filtered_a
        battery.percentage = msg.remaining
        battery.present = msg.connected
        self._state.battery = battery

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

    # --- Diagnósticos ---

    def _publish_diagnostics(self) -> None:
        now = self.get_clock().now().to_msg()
        statuses = []

        # VIO
        vio = DiagnosticStatus()
        vio.name = 'state_node: vio_status'
        vio.hardware_id = 'openvins'
        if self._state.vio_ok:
            vio.level = DiagnosticStatus.OK
            vio.message = 'VIO activo'
        else:
            vio.level = DiagnosticStatus.ERROR
            vio.message = 'VIO perdido — pose no fiable'
        statuses.append(vio)

        # GPS
        gps = DiagnosticStatus()
        gps.name = 'state_node: gps_status'
        gps.hardware_id = 'px4_gps'
        if self._state.gps_ok:
            gps.level = DiagnosticStatus.OK
            gps.message = 'GPS activo'
        else:
            gps.level = DiagnosticStatus.WARN
            gps.message = 'GPS no disponible'
        statuses.append(gps)

        # Batería
        bat = DiagnosticStatus()
        bat.name = 'state_node: battery'
        bat.hardware_id = 'px4_battery'
        pct = self._state.battery.percentage
        bat.values = [KeyValue(key='percentage', value=f'{pct:.2f}')]
        if pct <= BATTERY_ERROR_THRESHOLD:
            bat.level = DiagnosticStatus.ERROR
            bat.message = f'Bateria critica: {pct*100:.0f}%'
        elif pct <= BATTERY_WARN_THRESHOLD:
            bat.level = DiagnosticStatus.WARN
            bat.message = f'Bateria baja: {pct*100:.0f}%'
        else:
            bat.level = DiagnosticStatus.OK
            bat.message = f'Bateria OK: {pct*100:.0f}%'
        statuses.append(bat)

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