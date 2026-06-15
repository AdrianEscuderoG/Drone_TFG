"""
pilot_node.py

Piloto autónomo del dron. Implementa una máquina de estados que ejecuta
órdenes asíncronas recibidas en /drone/pilot_cmd (PilotCommand).

Comandos soportados (PilotCommand.command_type):
  TAKEOFF        — despegar hasta takeoff_altitude (0 = usar default_takeoff_alt)
  GOTO_RELATIVE  — moverse a [x, y, z, yaw] relativo al marco del dron
  GOTO_GLOBAL    — moverse a [x, y, z, yaw] en el marco global ENU (map)
  LAND           — aterrizar en la posición actual
  EMERGENCY      — abortar inmediatamente y aterrizar

Tras un despegue o un movimiento, el dron se queda estático (HOVERING) en el
punto de destino esperando la siguiente orden.

Interfaz:
  Sub  /drone/state         (DroneStatus)       — pose VIO y vio_ok
  Sub  /mavros/state        (mavros_msgs/State) — armado y modo de vuelo
  Sub  /drone/pilot_cmd     (PilotCommand)      — órdenes asíncronas
  Pub  /drone/cmd_vel       (TwistStamped)      — velocidades ENU (frame_id="map")
  Pub  /drone/pilot_status  (String)            — nombre del estado actual
  Cli  /mavros/cmd/takeoff  (CommandTOL)        — despegue inicial
  Cli  /mavros/set_mode     (SetMode)           — solicitar modo LAND
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import CommandTOL, SetMode
from std_msgs.msg import String

from drone_msgs.msg import DroneStatus, PilotCommand
from drone_pilot.pid_controller import PIDController

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class FlightState(Enum):
    IDLE = auto()
    TAKING_OFF = auto()
    NAVIGATING = auto()
    HOVERING = auto()
    LANDING = auto()
    EMERGENCY = auto()


class PilotNode(Node):

    CONTROL_HZ = 20.0

    def __init__(self):
        super().__init__('pilot_node')

        # --- Parámetros ajustables ---
        self.declare_parameter('kp_xy', 0.5)
        self.declare_parameter('ki_xy', 0.0)
        self.declare_parameter('kd_xy', 0.15)
        self.declare_parameter('kp_z', 0.6)
        self.declare_parameter('ki_z', 0.0)
        self.declare_parameter('kd_z', 0.15)
        self.declare_parameter('kp_yaw', 1.0)
        self.declare_parameter('max_vel_xy', 2.0)    # m/s
        self.declare_parameter('max_vel_z', 1.0)     # m/s
        self.declare_parameter('max_yaw_rate', 0.5)  # rad/s
        self.declare_parameter('pos_tol_xy', 0.5)    # m
        self.declare_parameter('pos_tol_z', 0.3)     # m
        self.declare_parameter('yaw_tol', 0.1)       # rad
        self.declare_parameter('default_takeoff_alt', 5.0)  # m

        kp_xy = self.get_parameter('kp_xy').value
        ki_xy = self.get_parameter('ki_xy').value
        kd_xy = self.get_parameter('kd_xy').value
        kp_z = self.get_parameter('kp_z').value
        ki_z = self.get_parameter('ki_z').value
        kd_z = self.get_parameter('kd_z').value
        kp_yaw = self.get_parameter('kp_yaw').value
        max_vel_xy = self.get_parameter('max_vel_xy').value
        max_vel_z = self.get_parameter('max_vel_z').value
        max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self._tol_xy = self.get_parameter('pos_tol_xy').value
        self._tol_z = self.get_parameter('pos_tol_z').value
        self._tol_yaw = self.get_parameter('yaw_tol').value
        self._default_takeoff_alt = self.get_parameter('default_takeoff_alt').value

        # --- Controladores PID ---
        self._pid_x = PIDController(kp_xy, ki_xy, kd_xy, -max_vel_xy, max_vel_xy)
        self._pid_y = PIDController(kp_xy, ki_xy, kd_xy, -max_vel_xy, max_vel_xy)
        self._pid_z = PIDController(kp_z, ki_z, kd_z, -max_vel_z, max_vel_z)
        self._pid_yaw = PIDController(kp_yaw, 0.0, 0.0, -max_yaw_rate, max_yaw_rate)

        # --- Estado interno ---
        self._state = FlightState.IDLE
        self._takeoff_alt = self._default_takeoff_alt
        self._land_mode_sent = False

        # Punto objetivo actual [x, y, z, yaw] en ENU (NAVIGATING/HOVERING)
        self._target = [0.0, 0.0, 0.0, 0.0]

        # Pose actual en ENU
        self._pos = [0.0, 0.0, 0.0]
        self._yaw = 0.0
        self._vio_ok = False

        # Estado MAVROS (no desde state_node, que usa tópicos PX4)
        self._mavros_armed = False
        self._mavros_mode = ''

        self._last_t = None

        # --- Suscripciones ---
        self.create_subscription(
            DroneStatus, '/drone/state', self._on_drone_state, RELIABLE_QOS)
        self.create_subscription(
            MavrosState, '/mavros/state', self._on_mavros_state, RELIABLE_QOS)
        self.create_subscription(
            PilotCommand, '/drone/pilot_cmd', self._on_command, RELIABLE_QOS)

        # --- Publicaciones ---
        self._pub_cmd = self.create_publisher(TwistStamped, '/drone/cmd_vel', RELIABLE_QOS)
        self._pub_status = self.create_publisher(String, '/drone/pilot_status', RELIABLE_QOS)

        # --- Clientes de servicio ---
        self._cli_takeoff = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self._cli_set_mode = self.create_client(SetMode, '/mavros/set_mode')

        # --- Timer de control ---
        self.create_timer(1.0 / self.CONTROL_HZ, self._control_loop)

        self.get_logger().info('pilot_node iniciado. Esperando órdenes en /drone/pilot_cmd...')

    # ------------------------------------------------------------------
    # Callbacks de suscripción
    # ------------------------------------------------------------------

    def _on_drone_state(self, msg: DroneStatus) -> None:
        p = msg.pose.pose.pose.position
        self._pos = [p.x, p.y, p.z]
        q = msg.pose.pose.pose.orientation
        self._yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        self._vio_ok = msg.vio_ok

        if (not self._vio_ok
                and self._state in (FlightState.TAKING_OFF,
                                    FlightState.NAVIGATING,
                                    FlightState.HOVERING)):
            self.get_logger().error('VIO perdido durante el vuelo. Emergencia.')
            self._switch(FlightState.EMERGENCY)

    def _on_mavros_state(self, msg: MavrosState) -> None:
        self._mavros_armed = msg.armed
        self._mavros_mode = msg.mode

    def _on_command(self, msg: PilotCommand) -> None:
        if msg.command_type == PilotCommand.TAKEOFF:
            self._cmd_takeoff(msg)
        elif msg.command_type == PilotCommand.GOTO_RELATIVE:
            self._cmd_goto_relative(msg)
        elif msg.command_type == PilotCommand.GOTO_GLOBAL:
            self._cmd_goto_global(msg)
        elif msg.command_type == PilotCommand.LAND:
            self._cmd_land()
        elif msg.command_type == PilotCommand.EMERGENCY:
            self.get_logger().warn('Orden de EMERGENCIA recibida.')
            self._switch(FlightState.EMERGENCY)
        else:
            self.get_logger().warn(
                f'Tipo de comando desconocido: {msg.command_type}')

    # ------------------------------------------------------------------
    # Manejadores de comandos
    # ------------------------------------------------------------------

    def _cmd_takeoff(self, msg: PilotCommand) -> None:
        if self._state != FlightState.IDLE:
            self.get_logger().warn(
                f'TAKEOFF ignorado: el piloto no está en IDLE ({self._state.name}).')
            return
        if not self._mavros_armed:
            self.get_logger().warn('TAKEOFF ignorado: el dron no está armado.')
            return
        self._takeoff_alt = (float(msg.takeoff_altitude)
                              if msg.takeoff_altitude > 0.0
                              else self._default_takeoff_alt)
        self._reset_pids()
        self._switch(FlightState.TAKING_OFF)

    def _cmd_goto_relative(self, msg: PilotCommand) -> None:
        if self._state not in (FlightState.HOVERING, FlightState.NAVIGATING):
            self.get_logger().warn(
                f'GOTO_RELATIVE ignorado: el piloto no está volando ({self._state.name}).')
            return
        # [x, y, z, yaw] relativos al marco del dron (x: adelante, y: izquierda)
        cos_y, sin_y = math.cos(self._yaw), math.sin(self._yaw)
        dx = msg.x * cos_y - msg.y * sin_y
        dy = msg.x * sin_y + msg.y * cos_y
        target = [
            self._pos[0] + dx,
            self._pos[1] + dy,
            self._pos[2] + msg.z,
            _norm_angle(self._yaw + msg.yaw),
        ]
        self._set_target(target)

    def _cmd_goto_global(self, msg: PilotCommand) -> None:
        if self._state not in (FlightState.HOVERING, FlightState.NAVIGATING):
            self.get_logger().warn(
                f'GOTO_GLOBAL ignorado: el piloto no está volando ({self._state.name}).')
            return
        target = [msg.x, msg.y, msg.z, _norm_angle(msg.yaw)]
        self._set_target(target)

    def _cmd_land(self) -> None:
        if self._state in (FlightState.IDLE, FlightState.LANDING, FlightState.EMERGENCY):
            return
        self.get_logger().info('Orden de aterrizaje recibida.')
        self._switch(FlightState.LANDING)

    def _set_target(self, target: list) -> None:
        self._target = target
        self._reset_pids()
        self.get_logger().info(
            f'Nuevo objetivo: x={target[0]:.2f} y={target[1]:.2f} '
            f'z={target[2]:.2f} yaw={target[3]:.2f}')
        self._switch(FlightState.NAVIGATING)

    # ------------------------------------------------------------------
    # Bucle de control (20 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        now = self.get_clock().now()
        if self._last_t is None:
            self._last_t = now
            return
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now

        s = String()
        s.data = self._state.name
        self._pub_status.publish(s)

        if self._state == FlightState.IDLE:
            self._cmd(0.0, 0.0, 0.0, 0.0)
        elif self._state == FlightState.TAKING_OFF:
            self._handle_takeoff()
        elif self._state == FlightState.NAVIGATING:
            self._handle_navigate(dt)
        elif self._state == FlightState.HOVERING:
            self._handle_hover(dt)
        elif self._state in (FlightState.LANDING, FlightState.EMERGENCY):
            self._handle_landing()

    # ------------------------------------------------------------------
    # Manejadores de estado
    # ------------------------------------------------------------------

    def _handle_takeoff(self) -> None:
        # ArduPilot gestiona el ascenso tras recibir MAV_CMD_NAV_TAKEOFF.
        # Solo monitorizamos altitud VIO y transicionamos al alcanzar el objetivo.
        err_z = self._takeoff_alt - self._pos[2]
        if abs(err_z) < self._tol_z:
            self.get_logger().info(
                f'Altitud de despegue alcanzada: {self._pos[2]:.2f} m')
            self._target = [self._pos[0], self._pos[1], self._takeoff_alt, self._yaw]
            self._reset_pids()
            self._switch(FlightState.HOVERING)

    def _handle_navigate(self, dt: float) -> None:
        ex, ey, ez, eyaw = self._position_errors()
        dist_xy = math.hypot(ex, ey)

        self._cmd(
            self._pid_x.compute(ex, dt),
            self._pid_y.compute(ey, dt),
            self._pid_z.compute(ez, dt),
            self._pid_yaw.compute(eyaw, dt),
        )

        if dist_xy < self._tol_xy and abs(ez) < self._tol_z and abs(eyaw) < self._tol_yaw:
            self.get_logger().info('Objetivo alcanzado. Esperando siguiente orden.')
            self._reset_pids()
            self._switch(FlightState.HOVERING)

    def _handle_hover(self, dt: float) -> None:
        # Mantiene la posición/orientación objetivo mediante correcciones PID
        # mientras se espera la siguiente orden.
        ex, ey, ez, eyaw = self._position_errors()
        self._cmd(
            self._pid_x.compute(ex, dt),
            self._pid_y.compute(ey, dt),
            self._pid_z.compute(ez, dt),
            self._pid_yaw.compute(eyaw, dt),
        )

    def _handle_landing(self) -> None:
        self._cmd(0.0, 0.0, 0.0, 0.0)

        if not self._land_mode_sent:
            self._request_land_mode()

        # ArduPilot desarmará automáticamente al tocar tierra
        if self._land_mode_sent and not self._mavros_armed:
            self.get_logger().info('Aterrizaje completado.')
            self._land_mode_sent = False
            self._state = FlightState.IDLE
            self.get_logger().info('Piloto en espera (IDLE).')

    # ------------------------------------------------------------------
    # Gestión de transiciones
    # ------------------------------------------------------------------

    def _switch(self, new: FlightState) -> None:
        if new == self._state:
            return
        self.get_logger().info(f'Estado: {self._state.name} → {new.name}')
        self._state = new

        if new == FlightState.TAKING_OFF:
            self._land_mode_sent = False
            self._call_takeoff()
        elif new in (FlightState.LANDING, FlightState.EMERGENCY):
            self._land_mode_sent = False

    def _call_takeoff(self) -> None:
        if not self._cli_takeoff.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Servicio /mavros/cmd/takeoff no disponible.')
            return
        req = CommandTOL.Request()
        req.altitude = float(self._takeoff_alt)
        future = self._cli_takeoff.call_async(req)
        future.add_done_callback(self._on_takeoff_response)

    def _on_takeoff_response(self, future) -> None:
        res = future.result()
        if res and res.success:
            self.get_logger().info(
                f'Despegue iniciado hacia {self._takeoff_alt:.1f} m.')
        else:
            self.get_logger().error(
                'Fallo en el comando de despegue. '
                'Comprueba que el dron esté armado en modo GUIDED.')

    def _request_land_mode(self) -> None:
        if not self._cli_set_mode.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Servicio set_mode no disponible aún.')
            return
        req = SetMode.Request()
        req.custom_mode = 'LAND'
        future = self._cli_set_mode.call_async(req)
        future.add_done_callback(self._on_land_mode_response)
        self._land_mode_sent = True

    def _on_land_mode_response(self, future) -> None:
        res = future.result()
        if res and res.mode_sent:
            self.get_logger().info('Modo LAND activado en ArduPilot.')
        else:
            self.get_logger().error('Fallo al activar modo LAND.')
            self._land_mode_sent = False  # reintentar en el próximo ciclo

    # ------------------------------------------------------------------
    # Utilidades
    # ------------------------------------------------------------------

    def _position_errors(self) -> tuple:
        ex = self._target[0] - self._pos[0]
        ey = self._target[1] - self._pos[1]
        ez = self._target[2] - self._pos[2]
        eyaw = _norm_angle(self._target[3] - self._yaw)
        return ex, ey, ez, eyaw

    def _cmd(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # velocidades en marco ENU local
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.z = yaw_rate
        self._pub_cmd.publish(msg)

    def _reset_pids(self) -> None:
        for pid in (self._pid_x, self._pid_y, self._pid_z, self._pid_yaw):
            pid.reset()


# ------------------------------------------------------------------
# Funciones auxiliares de geometría
# ------------------------------------------------------------------

def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def main(args=None):
    rclpy.init(args=args)
    node = PilotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
