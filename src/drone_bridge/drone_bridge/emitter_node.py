# emitter_node.py
#
# Puente de salida: recibe comandos del sistema y los envía a ArduPilot
# vía MAVROS.
#
# En esta versión mínima hace tres cosas verificables en SITL:
#   1. Cambia el modo de vuelo a GUIDED
#   2. Arma el dron
#   3. Envía comandos de velocidad de /drone/cmd_vel a MAVROS
#
# Secuencia de arranque obligatoria en ArduPilot:
#   modo GUIDED → armar → enviar setpoints
# Si mandas setpoints antes de armar, ArduPilot los ignora silenciosamente.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class EmitterNode(Node):
    """
    Suscribe a /drone/cmd_vel y lo reenvía a MAVROS.
    Expone dos métodos internos (set_mode, arm) que usa
    la secuencia de arranque automática.

    Flujo de arranque automático:
      Al recibir el primer heartbeat con connected=True:
        1. Llama a set_mode('GUIDED')
        2. Espera confirmación
        3. Llama a arm()
      Una vez armado, los mensajes de /drone/cmd_vel
      se reenvían a /mavros/setpoint_velocity/cmd_vel.
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

        # --- SUSCRIPCIONES ---

        # Estado del dron: necesitamos saber cuándo está conectado y armado
        self.sub_state = self.create_subscription(
            State,
            '/mavros/state',
            self.cb_state,
            10
        )

        # Comandos de velocidad del sistema propio
        self.sub_cmd_vel = self.create_subscription(
            TwistStamped,
            '/drone/cmd_vel',
            self.cb_cmd_vel,
            cmd_qos
        )

        # Recibe la odometría de OpenVINS
        self.sub_vio = self.create_subscription(
            Odometry,
            '/ov_msckf/odomimu',
            self.cb_vio,
            10
        )

        # --- PUBLICACIONES ---

        # MAVROS acepta velocidades en este topic cuando el modo es GUIDED
        self.pub_vel = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            cmd_qos
        )

        # Envía la pose a ArduPilot — MAVROS la convierte a VISION_POSITION_ESTIMATE
        self.pub_vision = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

        # --- CLIENTES DE SERVICIO ---

        # Servicio para armar/desarmar
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        # Servicio para cambiar modo de vuelo
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # --- ESTADO INTERNO ---

        self.connected = False
        self.armed = False
        self.mode = ''
        self.arming_requested = False  # evita llamar arm() múltiples veces

        self.get_logger().info('emitter_node iniciado — esperando conexión FCU')

    # ------------------------------------------------------------------
    # CALLBACK ESTADO
    # Cuando MAVROS confirma conexión, lanza la secuencia de arranque.
    # ------------------------------------------------------------------
    def cb_state(self, msg: State):
        prev_connected = self.connected
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

        # Primera vez que se conecta: lanza secuencia automática
        if self.connected and not prev_connected:
            self.get_logger().info('FCU conectado — iniciando secuencia de arranque')
            self.set_mode('GUIDED')

        # Log cuando cambia el estado de armado
        if msg.armed and not self.arming_requested:
            self.get_logger().info('Dron ARMADO — listo para recibir comandos')

    # ------------------------------------------------------------------
    # CALLBACK CMD_VEL
    # Solo reenvía si el dron está armado. Si no, avisa en el log.
    # ------------------------------------------------------------------
    '''def cb_cmd_vel(self, msg: TwistStamped):
        if not self.armed:
            self.get_logger().warn(
                'cmd_vel recibido pero dron NO armado — ignorando',
                throttle_duration_sec=5.0  # máximo un warning cada 5s
            )
            return

        # Actualiza el timestamp antes de reenviar
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_vel.publish(msg)'''
    def cb_cmd_vel(self, msg: TwistStamped):
    # Reenviar siempre que estemos en modo GUIDED, armado o no
    # ArduPilot necesita setpoints continuos para mantener el armado
        if self.mode != 'GUIDED':
            return
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_vel.publish(msg)

    def cb_vio(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'  # ArduPilot espera este frame_id
        pose.pose = msg.pose.pose
        self.pub_vision.publish(pose)

    # ------------------------------------------------------------------
    # CAMBIO DE MODO
    # ArduPilot requiere modo GUIDED para aceptar setpoints externos.
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
            self.get_logger().info(f'Modo {mode} activado — armando dron')
            self.arm()
        else:
            self.get_logger().error(f'Fallo al activar modo {mode}')

    # ------------------------------------------------------------------
    # ARMADO
    # Solo se puede armar en modo GUIDED y con prearm checks OK.
    # En SITL los prearm checks están relajados por defecto.
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
