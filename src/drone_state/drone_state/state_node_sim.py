"""
state_node_sim.py

Agrega datos de sensores y estado del FCU en un único topic /drone/state
(DroneStatus). Compatible con ArduPilot (vía MAVROS) y PX4 (vía uXRCE-DDS),
y con dos fuentes de odometría distintas según el entorno.

Parámetros ROS 2
────────────────
  odometry_source   'mavros_local' → /mavros/local_position/odom  (SITL, ground truth)
                    'openvins'     → /ov_msckf/odomimu             (hardware real + VIO)
                    Default: 'mavros_local'

  flight_controller 'ardupilot'   → /mavros/state                 (armed + mode)
                    'px4'         → /fmu/out/vehicle_status + battery_status
                    Default: 'ardupilot'

Publicaciones
─────────────
  /drone/state    drone_msgs/DroneStatus        @ 10 Hz
  /diagnostics    diagnostic_msgs/DiagnosticArray

Por qué dos parámetros en lugar de dos nodos separados
───────────────────────────────────────────────────────
  La lógica de watchdog, diagnósticos y publicación es idéntica en todos
  los casos. Duplicar el fichero crearía deuda de mantenimiento: cualquier
  corrección habría que aplicarla en varios sitios. Los parámetros permiten
  seleccionar el backend en el launch file sin cambiar el código.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, BatteryState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from drone_msgs.msg import DroneStatus

# ── Imports opcionales ────────────────────────────────────────────────────────
# Se hacen aquí para detectar el problema en el arranque del nodo y dar
# un mensaje de error claro, en lugar de un traceback críptico.

try:
    from mavros_msgs.msg import State as MavrosState
    _MAVROS_AVAILABLE = True
except ImportError:
    _MAVROS_AVAILABLE = False

try:
    from px4_msgs.msg import VehicleStatus, BatteryStatus as PX4BatteryStatus
    _PX4_AVAILABLE = True
except ImportError:
    _PX4_AVAILABLE = False

# ── Constantes ────────────────────────────────────────────────────────────────

VIO_TIMEOUT_SEC       = 0.5   # si no llega odometría en 500 ms → vio_ok = False
GPS_TIMEOUT_SEC       = 2.0   # si no llega NavSatFix en 2 s   → gps_ok = False

BATTERY_WARN_THRESHOLD  = 0.20
BATTERY_ERROR_THRESHOLD = 0.10

# Mapa de nav_state numérico (PX4) → nombre legible
NAV_STATE_NAMES = {
    0:  'MANUAL',       1:  'ALTCTL',      2:  'POSCTL',
    3:  'AUTO_MISSION', 4:  'AUTO_LOITER', 5:  'AUTO_RTL',
    10: 'ACRO',         12: 'DESCEND',     13: 'TERMINATION',
    14: 'OFFBOARD',     15: 'STABILIZED',  17: 'AUTO_TAKEOFF',
    18: 'AUTO_LAND',    21: 'ORBIT',
}

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

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)


# ── Nodo ──────────────────────────────────────────────────────────────────────

class StateNode(Node):

    def __init__(self):
        super().__init__('state_node')

        # ── Parámetros ────────────────────────────────────────────────────
        self.declare_parameter('odometry_source',   'mavros_local')
        self.declare_parameter('flight_controller', 'ardupilot')

        self._odom_src = self.get_parameter('odometry_source').value
        self._fc       = self.get_parameter('flight_controller').value

        self.get_logger().info(
            f'Configuración → odometry_source={self._odom_src!r}  '
            f'flight_controller={self._fc!r}'
        )

        # ── Estado interno ────────────────────────────────────────────────
        self._drone_state    = DroneStatus()
        self._last_odom_time = None   # marca temporal del último mensaje de pose
        self._last_gps_time  = None

        # Batería: valor inicial 100 % mientras no llegue dato real.
        # En SITL, /mavros/battery puede no publicar nada.
        self._drone_state.battery.percentage = 1.0

        # ── Suscripción a la fuente de odometría ──────────────────────────
        if self._odom_src == 'mavros_local':
            # Ground truth del simulador: ArduPilot SITL publica la posición
            # real del vehículo aquí. Ideal para validar el piloto sin VIO.
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
            # Estimación VIO de OpenVINS. Mismo tipo de mensaje (Odometry),
            # distinto topic. El callback _cb_odom es idéntico en ambos casos.
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

        # ── Suscripciones al FCU ──────────────────────────────────────────
        if self._fc == 'ardupilot':
            if not _MAVROS_AVAILABLE:
                self.get_logger().fatal(
                    'flight_controller="ardupilot" requiere mavros_msgs, '
                    'pero no está instalado en este entorno.'
                )
                raise RuntimeError('mavros_msgs no disponible')

            # /mavros/state: armed, mode, connected — publicado a ~1 Hz
            # Usa QoS RELIABLE porque MAVROS lo configura así en este topic.
            self.create_subscription(
                MavrosState,
                '/mavros/state',
                self._cb_mavros_state,
                RELIABLE_QOS,
            )

            # /mavros/battery: puede no existir en SITL (no pasa nada,
            # el valor inicial 100 % se mantiene si no llega ningún mensaje).
            self.create_subscription(
                BatteryState,
                '/mavros/battery',
                self._cb_mavros_battery,
                SENSOR_QOS,
            )
            self.get_logger().info(
                'FCU: ArduPilot — /mavros/state + /mavros/battery'
            )

        elif self._fc == 'px4':
            if not _PX4_AVAILABLE:
                self.get_logger().fatal(
                    'flight_controller="px4" requiere px4_msgs, '
                    'pero no está instalado en este entorno.'
                )
                raise RuntimeError('px4_msgs no disponible')

            self.create_subscription(
                VehicleStatus,
                '/fmu/out/vehicle_status',
                self._cb_vehicle_status,
                PX4_QOS,
            )
            self.create_subscription(
                PX4BatteryStatus,
                '/fmu/out/battery_status',
                self._cb_px4_battery,
                PX4_QOS,
            )
            self.get_logger().info(
                'FCU: PX4 — /fmu/out/vehicle_status + /fmu/out/battery_status'
            )

        else:
            self.get_logger().fatal(
                f'Valor desconocido para flight_controller: {self._fc!r}. '
                'Valores válidos: "ardupilot", "px4".'
            )
            raise ValueError(f'flight_controller inválido: {self._fc!r}')

        # ── Suscripción común (GPS watchdog) ──────────────────────────────
        # receiver_node_ardu republica /mavros/global_position/global aquí.
        # Solo se usa para actualizar el flag gps_ok; el piloto no usa GPS.
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
    # Callbacks: fuente de odometría
    # ────────────────────────────────────────────────────────────────────────

    def _cb_odom(self, msg: Odometry) -> None:
        """
        Callback compartido para mavros_local y openvins.
        Ambos publican nav_msgs/Odometry con la misma estructura de campos.
        La diferencia está solo en el topic y en el frame_id del header,
        pero pilot_node trabaja con valores numéricos, no con frames.

        Campos mapeados:
          msg.pose        → DroneStatus.pose.pose     (PoseWithCovariance)
          msg.twist.twist → DroneStatus.velocity.twist (Twist, sin covarianza)
        """
        self._last_odom_time = self.get_clock().now()
        self._drone_state.pose.header       = msg.header
        self._drone_state.pose.pose         = msg.pose       # PoseWithCovariance
        self._drone_state.velocity.header   = msg.header
        self._drone_state.velocity.twist    = msg.twist.twist  # Twist

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks: GPS
    # ────────────────────────────────────────────────────────────────────────

    def _cb_gps(self, msg: NavSatFix) -> None:
        # status.status < 0 significa sin fix; no actualizamos el watchdog.
        if msg.status.status >= 0:
            self._last_gps_time = self.get_clock().now()

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks: ArduPilot (MAVROS)
    # ────────────────────────────────────────────────────────────────────────

    def _cb_mavros_state(self, msg) -> None:
        """
        mavros_msgs/State → DroneStatus.armed + DroneStatus.flight_mode.
        Se loguea cada cambio de modo o de armado para facilitar el debug.
        """
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
        """
        sensor_msgs/BatteryState de /mavros/battery.
        Si el topic no existe en SITL, este callback nunca se llama
        y el valor inicial (100 %) se mantiene. Silencioso por diseño.
        """
        self._drone_state.battery = msg

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks: PX4 (uXRCE-DDS)
    # ────────────────────────────────────────────────────────────────────────

    def _cb_vehicle_status(self, msg) -> None:
        """
        px4_msgs/VehicleStatus → armed, flight_mode.
        ARMING_STATE_ARMED = 2 en la enumeración de PX4.
        """
        self._drone_state.armed = (msg.arming_state == 2)
        self._drone_state.flight_mode = NAV_STATE_NAMES.get(
            msg.nav_state, f'UNKNOWN_{msg.nav_state}'
        )

    def _cb_px4_battery(self, msg) -> None:
        """
        px4_msgs/BatteryStatus → sensor_msgs/BatteryState.
        Conversión de campos equivalentes entre los dos tipos de mensaje.
        """
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.voltage     = msg.voltage_filtered_v
        battery.current     = msg.current_filtered_a
        battery.percentage  = msg.remaining
        battery.present     = msg.connected
        self._drone_state.battery = battery

    # ────────────────────────────────────────────────────────────────────────
    # Watchdog de fuentes de datos
    # ────────────────────────────────────────────────────────────────────────

    def _check_watchdogs(self) -> None:
        """
        Comprueba si han llegado mensajes recientes de pose y GPS.
        Si alguna fuente supera su timeout, se activa el flag correspondiente
        y se emite un warning (solo en el cambio, no en cada tick).
        """
        now = self.get_clock().now()

        # ── Pose / VIO ────────────────────────────────────────────────────
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

        # ── GPS ───────────────────────────────────────────────────────────
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
        if self._drone_state.vio_ok:
            pose_st.level   = DiagnosticStatus.OK
            pose_st.message = 'Pose activa'
        else:
            pose_st.level   = DiagnosticStatus.ERROR
            pose_st.message = 'Sin odometría — pose no fiable'
        statuses.append(pose_st)

        # GPS
        gps_st = DiagnosticStatus()
        gps_st.name        = 'state_node: gps'
        gps_st.hardware_id = 'gps'
        if self._drone_state.gps_ok:
            gps_st.level   = DiagnosticStatus.OK
            gps_st.message = 'GPS activo'
        else:
            gps_st.level   = DiagnosticStatus.WARN
            gps_st.message = 'GPS no disponible (normal en SITL sin GPS)'
        statuses.append(gps_st)

        # Batería
        pct    = self._drone_state.battery.percentage
        bat_st = DiagnosticStatus()
        bat_st.name        = 'state_node: battery'
        bat_st.hardware_id = self._fc
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
        arr.status = statuses
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