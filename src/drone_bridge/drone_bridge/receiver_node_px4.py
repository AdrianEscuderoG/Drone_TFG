# receiver_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Tipos PX4 (entrada)
from px4_msgs.msg import SensorCombined
from px4_msgs.msg import VehicleGlobalPosition
from px4_msgs.msg import BatteryStatus
from px4_msgs.msg import VehicleStatus

# Tipos ROS 2 estándar (salida)
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header
import math


class ReceiverNode(Node):
    """
    Puente de entrada: convierte topics PX4 (/fmu/out/*)
    a topics ROS 2 estándar (/drone/*) que entiende el
    resto del sistema (state_node, OpenVINS, etc.)
    """

    def __init__(self):
        super().__init__('receiver_node')

        # QoS que usa PX4 para publicar: BEST_EFFORT + VOLATILE
        # Si no coincide con este perfil, no recibirás nada
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS para nuestras publicaciones internas: RELIABLE
        # El state_node necesita fiabilidad en los datos de estado
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- SUSCRIPCIONES A PX4 ---

        self.sub_imu = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.cb_imu,
            px4_qos
        )

        self.sub_gps = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self.cb_gps,
            px4_qos
        )

        self.sub_battery = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.cb_battery,
            px4_qos
        )

        self.sub_status = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.cb_status,
            px4_qos
        )

        # --- PUBLICACIONES EN TOPICS PROPIOS ---

        self.pub_imu = self.create_publisher(
            Imu,
            '/drone/imu',
            out_qos
        )

        self.pub_gps = self.create_publisher(
            NavSatFix,
            '/drone/gps',
            out_qos
        )

        self.pub_battery = self.create_publisher(
            BatteryState,
            '/drone/battery',
            out_qos
        )

        # El estado del vehículo lo guardamos como parámetro
        # y lo publicamos en /drone/armed y /drone/flight_mode
        # para que el pilot_node sepa cuándo puede mandar comandos
        self.armed = False
        self.nav_state = 0

        self.get_logger().info('receiver_node iniciado — escuchando /fmu/out/*')

    # ------------------------------------------------------------------
    # CALLBACK IMU
    # SensorCombined de PX4 contiene acelerómetro + giroscopio
    # Lo convertimos a sensor_msgs/Imu estándar de ROS 2
    # ------------------------------------------------------------------
    def cb_imu(self, msg: SensorCombined):
        imu_out = Imu()

        # Header con timestamp actual del sistema
        imu_out.header = Header()
        imu_out.header.stamp = self.get_clock().now().to_msg()
        imu_out.header.frame_id = 'imu_link'

        # Aceleración lineal (m/s²) — PX4 usa NED, ROS 2 usa ENU
        # Conversión NED→ENU: x=y_ned, y=x_ned, z=-z_ned
        imu_out.linear_acceleration.x =  msg.accelerometer_m_s2[1]
        imu_out.linear_acceleration.y =  msg.accelerometer_m_s2[0]
        imu_out.linear_acceleration.z = -msg.accelerometer_m_s2[2]

        # Velocidad angular (rad/s) — misma conversión NED→ENU
        imu_out.angular_velocity.x =  msg.gyro_rad[1]
        imu_out.angular_velocity.y =  msg.gyro_rad[0]
        imu_out.angular_velocity.z = -msg.gyro_rad[2]

        # Covarianzas desconocidas → -1 (convención ROS 2)
        imu_out.orientation_covariance[0] = -1.0
        imu_out.linear_acceleration_covariance[0] = -1.0
        imu_out.angular_velocity_covariance[0] = -1.0

        self.pub_imu.publish(imu_out)

    # ------------------------------------------------------------------
    # CALLBACK GPS
    # VehicleGlobalPosition → sensor_msgs/NavSatFix
    # ------------------------------------------------------------------
    def cb_gps(self, msg: VehicleGlobalPosition):
        gps_out = NavSatFix()

        gps_out.header = Header()
        gps_out.header.stamp = self.get_clock().now().to_msg()
        gps_out.header.frame_id = 'gps_link'

        gps_out.latitude  = msg.lat   # grados decimales
        gps_out.longitude = msg.lon   # grados decimales
        gps_out.altitude  = msg.alt   # metros sobre el nivel del mar

        # STATUS_FIX = 0 (señal GPS normal)
        gps_out.status.status = 0
        gps_out.status.service = 1  # SERVICE_GPS

        # Covarianza posicional (diagonal) en metros²
        # PX4 da EPH (error horizontal) y EPV (error vertical)
        h_var = msg.eph * msg.eph
        v_var = msg.epv * msg.epv
        gps_out.position_covariance = [
            h_var, 0.0,   0.0,
            0.0,   h_var, 0.0,
            0.0,   0.0,   v_var
        ]
        gps_out.position_covariance_type = 2  # COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub_gps.publish(gps_out)

    # ------------------------------------------------------------------
    # CALLBACK BATERÍA
    # BatteryStatus de PX4 → sensor_msgs/BatteryState
    # ------------------------------------------------------------------
    def cb_battery(self, msg: BatteryStatus):
        bat_out = BatteryState()

        bat_out.header = Header()
        bat_out.header.stamp = self.get_clock().now().to_msg()

        bat_out.voltage    = msg.voltage_v          # voltios
        bat_out.current    = -msg.current_a         # amperios (PX4 invierte signo)
        bat_out.percentage = msg.remaining          # 0.0 – 1.0

        # Umbrales de alerta — ajusta según tu batería real
        # El state_node los usará para publicar diagnósticos
        if msg.remaining < 0.20:
            self.get_logger().warn(
                f'BATERÍA BAJA: {msg.remaining*100:.0f}% — '
                f'{msg.voltage_v:.2f}V'
            )

        self.pub_battery.publish(bat_out)

    # ------------------------------------------------------------------
    # CALLBACK ESTADO DEL VEHÍCULO
    # VehicleStatus contiene: armado, modo de navegación, etc.
    # Lo guardamos internamente — el emitter lo necesitará
    # ------------------------------------------------------------------
    def cb_status(self, msg: VehicleStatus):
        prev_armed = self.armed
        self.armed     = msg.arming_state == 2  # ARMING_STATE_ARMED = 2
        self.nav_state = msg.nav_state

        # Log solo cuando cambia el estado para no saturar la consola
        if self.armed != prev_armed:
            estado = 'ARMADO' if self.armed else 'DESARMADO'
            self.get_logger().info(f'Dron {estado} — nav_state={self.nav_state}')


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
