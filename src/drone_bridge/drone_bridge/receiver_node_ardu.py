# receiver_node.py
#
# Puente de entrada: convierte topics de MAVROS (/mavros/*)
# a topics ROS 2 estándar propios del proyecto (/drone/*)
#
# Stack: ArduPilot SITL + MAVROS (NO PX4)
# Coordinadas: MAVROS ya publica en ENU — sin conversión necesaria

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Tipos de entrada (MAVROS publica estos)
from sensor_msgs.msg import Imu, NavSatFix
from mavros_msgs.msg import State
from std_msgs.msg import Float32

# Tipos de salida (topics propios del proyecto)
from sensor_msgs.msg import Imu as ImuOut
from sensor_msgs.msg import NavSatFix as NavSatFixOut


class ReceiverNode(Node):
    """
    Suscribe a MAVROS y republica en topics propios del proyecto.

    Entradas  (MAVROS):
      /mavros/imu/data          sensor_msgs/Imu       @ 10 Hz
      /mavros/global_position/global  sensor_msgs/NavSatFix @ 10 Hz
      /mavros/state             mavros_msgs/State     @  1 Hz

    Salidas (proyecto):
      /drone/imu                sensor_msgs/Imu
      /drone/gps                sensor_msgs/NavSatFix
      /drone/battery_percentage std_msgs/Float32  (simulado: siempre 100%)

    Por qué separar los topics:
      Si en el futuro cambias MAVROS por otro driver, solo modificas
      este nodo. El resto del sistema (state_node, OpenVINS, pilot)
      nunca ve topics de MAVROS directamente.
    """

    def __init__(self):
        super().__init__('receiver_node')

        # QoS de entrada: BEST_EFFORT para igualar lo que publica MAVROS.
        # MAVROS usa BEST_EFFORT en sensores de alta frecuencia.
        # Si usas RELIABLE aquí, ROS 2 descarta los mensajes por
        # incompatibilidad de QoS y no recibes nada.
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS de salida: RELIABLE para que state_node y OpenVINS
        # reciban todos los mensajes sin pérdidas.
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- SUSCRIPCIONES A MAVROS ---

        self.sub_imu = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.cb_imu,
            mavros_qos
        )

        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.cb_gps,
            mavros_qos
        )

        # State tiene QoS RELIABLE en MAVROS (es de baja frecuencia)
        self.sub_state = self.create_subscription(
            State,
            '/mavros/state',
            self.cb_state,
            10  # depth=10, RELIABLE por defecto
        )

        # --- PUBLICACIONES EN TOPICS PROPIOS ---

        self.pub_imu = self.create_publisher(Imu, '/drone/imu', out_qos)
        self.pub_gps = self.create_publisher(NavSatFix, '/drone/gps', out_qos)
        self.pub_battery = self.create_publisher(Float32, '/drone/battery_percentage', out_qos)

        # Estado interno del dron (actualizado por cb_state)
        self.connected = False
        self.armed = False
        self.mode = ''

        # Timer para publicar batería simulada a 1 Hz.
        # SITL no simula batería real; cuando tengas el dron físico
        # sustituye esto por una suscripción a /mavros/battery.
        self.create_timer(1.0, self.publish_battery)

        self.get_logger().info('receiver_node iniciado')

    # ------------------------------------------------------------------
    # CALLBACK IMU
    # MAVROS publica sensor_msgs/Imu directamente en ENU.
    # Solo necesitamos reenviar el mensaje sin modificación.
    # ------------------------------------------------------------------
    def cb_imu(self, msg: Imu):
        # Republicar tal cual — frame_id ya es correcto (base_link ENU)
        self.pub_imu.publish(msg)

    # ------------------------------------------------------------------
    # CALLBACK GPS
    # MAVROS publica sensor_msgs/NavSatFix con lat/lon/alt estándar.
    # ------------------------------------------------------------------
    def cb_gps(self, msg: NavSatFix):
        self.pub_gps.publish(msg)

    # ------------------------------------------------------------------
    # CALLBACK ESTADO
    # mavros_msgs/State contiene: connected, armed, mode, system_status
    # Lo usamos para logging y para que otros nodos consulten el estado
    # a través de los atributos de esta instancia.
    # ------------------------------------------------------------------
    def cb_state(self, msg: State):
        # Log solo cuando cambia algo relevante
        if msg.armed != self.armed:
            self.get_logger().info(
                f'Dron {"ARMADO" if msg.armed else "DESARMADO"}'
            )
        if msg.mode != self.mode:
            self.get_logger().info(f'Modo de vuelo: {msg.mode}')
        if msg.connected != self.connected:
            self.get_logger().info(
                f'FCU {"conectado" if msg.connected else "DESCONECTADO"}'
            )

        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

    # ------------------------------------------------------------------
    # BATERÍA SIMULADA
    # SITL no publica /mavros/battery con datos reales.
    # Publicamos 100% hasta tener hardware real.
    # ------------------------------------------------------------------
    def publish_battery(self):
        msg = Float32()
        msg.data = 100.0
        self.pub_battery.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
