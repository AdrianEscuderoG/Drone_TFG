# emitter_node.py
#
# Puente de salida: recibe comandos del sistema y los envía a ArduPilot
# vía MAVROS.
#
# Responsabilidades:
#   1. Reenviar comandos de velocidad de /drone/cmd_vel a MAVROS
#   2. Reenviar la pose VIO a ArduPilot
#   3. Exponer servicios al pilot_node para armar, cambiar modo y despegar
#
# El pilot_node nunca habla con MAVROS directamente — todo pasa por aquí.
#
# Secuencia de arranque en ArduPilot:
#   modo GUIDED → armar → enviar setpoints
# Si mandas setpoints antes de armar, ArduPilot los ignora silenciosamente.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

from drone_msgs.srv import ArmDrone, SetFlightMode, TakeoffDrone


class EmitterNode(Node):
    """
    Puente de salida hacia MAVROS. Centraliza toda la comunicación MAVLink
    para que ningún otro nodo del sistema dependa de MAVROS directamente.

    Parámetros ROS 2:
      auto_arm (bool, default False):
        Si True, arma el dron automáticamente al conectar con el FCU.
        Solo activar en SITL. En hardware real, el armado lo gestiona
        pilot_node como parte de la secuencia de TAKEOFF.

    Servicios expuestos (para pilot_node):
      /drone/cmd/arm      (ArmDrone)      — armar o desarmar
      /drone/cmd/mode     (SetFlightMode) — cambiar modo de vuelo
      /drone/cmd/takeoff  (TakeoffDrone)  — ejecutar despegue
    """

    def __init__(self):
        super().__init__('emitter_node')

        # QoS RELIABLE para comandos — no queremos perder ninguno
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- PARÁMETROS ---
        # auto_arm=False por defecto: el armado lo decide pilot_node,
        # no el momento de conexión. Actívalo solo en SITL.
        self.declare_parameter('auto_arm', False)
        self.auto_arm = self.get_parameter('auto_arm').value

        # --- ESTADO INTERNO ---
        self.connected        = False
        self.armed            = False
        self.mode             = ''
        self.arming_requested = False  # evita doble armado en flujo auto_arm

        # --- SUSCRIPCIONES ---

        # Estado del FCU: detectar conexión y cambios de armado/modo
        self.sub_state = self.create_subscription(
            State, '/mavros/state', self.cb_state, 10)

        # Comandos de velocidad del pilot_node
        self.sub_cmd_vel = self.create_subscription(
            TwistStamped, '/drone/cmd_vel', self.cb_cmd_vel, cmd_qos)

        # Odometría de OpenVINS → pose para ArduPilot
        self.sub_vio = self.create_subscription(
            Odometry, '/ov_msckf/odomimu', self.cb_vio, 10)

        # --- PUBLICACIONES ---

        # Setpoints de velocidad a MAVROS (modo GUIDED)
        self.pub_vel = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', cmd_qos)

        # Pose VIO a ArduPilot vía MAVROS (VISION_POSITION_ESTIMATE)
        self.pub_vision = self.create_publisher(
            PoseStamped, '/mavros/vision_pose/pose', 10)

        # --- CLIENTES MAVROS ---

        self.arming_client  = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client    = self.create_client(SetMode,     '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL,  '/mavros/cmd/takeoff')

        # --- SERVIDORES DE SERVICIO (interfaz hacia pilot_node) ---

        self.create_service(ArmDrone,      '/drone/cmd/arm',     self._handle_arm)
        self.create_service(SetFlightMode, '/drone/cmd/mode',    self._handle_set_mode)
        self.create_service(TakeoffDrone,  '/drone/cmd/takeoff', self._handle_takeoff)

        self.get_logger().info(
            f'emitter_node iniciado — auto_arm={self.auto_arm} — esperando conexión FCU'
        )

    # ------------------------------------------------------------------
    # CALLBACK ESTADO
    # ------------------------------------------------------------------
    def cb_state(self, msg: State):
        prev_connected = self.connected
        self.connected = msg.connected
        self.armed     = msg.armed
        self.mode      = msg.mode

        if self.connected and not prev_connected:
            if self.auto_arm:
                self.get_logger().info('FCU conectado — auto_arm activo, iniciando secuencia')
                self.set_mode('GUIDED')
            else:
                self.get_logger().info(
                    'FCU conectado — auto_arm desactivado, '
                    'esperando orden de despegue'
                )

        if msg.armed and not self.arming_requested:
            self.get_logger().info('Dron ARMADO — listo para recibir comandos')

    # ------------------------------------------------------------------
    # CALLBACK CMD_VEL
    # Reenvía velocidades a MAVROS solo si estamos en modo GUIDED.
    # ArduPilot necesita setpoints continuos para mantener el armado.
    # ------------------------------------------------------------------
    def cb_cmd_vel(self, msg: TwistStamped):
        if self.mode != 'GUIDED':
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_vel.publish(msg)

    # ------------------------------------------------------------------
    # CALLBACK VIO
    # Republica la pose de OpenVINS hacia ArduPilot.
    # ------------------------------------------------------------------
    def cb_vio(self, msg: Odometry):
        pose = PoseStamped()
        pose.header          = msg.header
        pose.header.frame_id = 'map'  # ArduPilot espera este frame_id
        pose.pose            = msg.pose.pose
        self.pub_vision.publish(pose)

    # ------------------------------------------------------------------
    # CAMBIO DE MODO (interno — usado por auto_arm y _handle_set_mode)
    # ------------------------------------------------------------------
    def set_mode(self, mode: str):
        if not self.mode_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Servicio set_mode no disponible')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        future.add_done_callback(lambda f: self._on_mode_set(f, mode))

    def _on_mode_set(self, future, mode: str):
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f'Modo {mode} activado')
            # Solo en el flujo auto_arm se arma automáticamente aquí.
            # Cuando el pilot_node llama a /drone/cmd/mode, gestiona
            # el armado él mismo con /drone/cmd/arm.
            if self.auto_arm:
                self.arm()
        else:
            self.get_logger().error(f'Fallo al activar modo {mode}')

    # ------------------------------------------------------------------
    # ARMADO (interno — usado por auto_arm y _handle_arm)
    # ------------------------------------------------------------------
    def arm(self):
        if self.arming_requested:
            return
        self.arming_requested = True
        if not self.arming_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Servicio arming no disponible')
            return
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        future.add_done_callback(self._on_arm_response)

    def _on_arm_response(self, future):
        if future.result() and future.result().success:
            self.get_logger().info('Comando de armado enviado correctamente')
        else:
            self.get_logger().error(
                'Fallo al armar — revisa prearm checks en MAVProxy '
                '(ejecuta "arm throttle" para forzar en SITL)'
            )
            self.arming_requested = False  # permite reintentar

    # ------------------------------------------------------------------
    # SERVICIOS PARA PILOT_NODE
    # Reciben la petición del pilot y la reenvían a MAVROS.
    # La respuesta es inmediata — la confirmación real llega por
    # /mavros/state (armed, mode) que el pilot también escucha.
    # ------------------------------------------------------------------

    def _handle_arm(self, request, response):
        if not self.arming_client.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = 'Servicio MAVROS arming no disponible'
            return response

        req = CommandBool.Request()
        req.value = request.arm  # True = armar, False = desarmar
        future = self.arming_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Armado: {"OK" if f.result() and f.result().success else "FALLO"}'
            )
        )
        response.success = True
        response.message = 'Petición de armado enviada'
        return response

    def _handle_set_mode(self, request, response):
        if not self.mode_client.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = 'Servicio MAVROS set_mode no disponible'
            return response

        req = SetMode.Request()
        req.custom_mode = request.mode
        future = self.mode_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Modo {request.mode}: '
                f'{"OK" if f.result() and f.result().mode_sent else "FALLO"}'
            )
        )
        response.success = True
        response.message = f'Petición de modo {request.mode} enviada'
        return response

    def _handle_takeoff(self, request, response):
        if not self.takeoff_client.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = 'Servicio MAVROS cmd/takeoff no disponible'
            return response

        req = CommandTOL.Request()
        req.altitude = request.altitude
        future = self.takeoff_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Takeoff a {request.altitude:.1f} m: '
                f'{"OK" if f.result() and f.result().success else "FALLO"}'
            )
        )
        response.success = True
        response.message = f'Petición de despegue a {request.altitude:.1f} m enviada'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = EmitterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()