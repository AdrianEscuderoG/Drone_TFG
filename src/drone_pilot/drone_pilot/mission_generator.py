"""
mission_generator.py

Genera una misión de escaneo en grid (lawnmower) y la publica en /drone/mission.

Uso típico en SITL:
  ros2 run drone_pilot mission_generator \\
      --ros-args -p field_width:=50.0 -p field_height:=50.0 \\
                 -p altitude:=10.0 -p lane_spacing:=8.0

Parámetros:
  field_width       Ancho del campo en metros (eje X / Este)
  field_height      Alto del campo en metros (eje Y / Norte)
  altitude          Altitud de vuelo sobre el origen (metros)
  lane_spacing      Distancia entre pasadas (metros)
  origin_x/y        Esquina SW del campo en el marco ENU (metros)
  takeoff_altitude  Altitud de despegue (metros)
  loiter_time       Segundos de hover al final de cada pasada
  capture_image     Si se registra captura en cada waypoint de fin de pasada
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped

from drone_msgs.msg import MissionPlan, MissionWaypoint

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class MissionGeneratorNode(Node):

    def __init__(self):
        super().__init__('mission_generator')

        self.declare_parameter('field_width', 50.0)
        self.declare_parameter('field_height', 50.0)
        self.declare_parameter('altitude', 10.0)
        self.declare_parameter('lane_spacing', 8.0)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('takeoff_altitude', 5.0)
        self.declare_parameter('loiter_time', 3.0)
        self.declare_parameter('capture_image', True)

        self._pub = self.create_publisher(MissionPlan, '/drone/mission', RELIABLE_QOS)

        # Publica la misión 2 s después del arranque para que el piloto esté listo
        self._timer = self.create_timer(2.0, self._publish_once)

    def _publish_once(self) -> None:
        self._timer.cancel()

        waypoints = self._build_grid()
        msg = MissionPlan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.waypoints = waypoints
        msg.takeoff_altitude = float(self.get_parameter('takeoff_altitude').value)

        self._pub.publish(msg)
        self.get_logger().info(
            f'Misión publicada: {len(waypoints)} waypoints | '
            f'altitud {self.get_parameter("altitude").value:.0f} m | '
            f'separación {self.get_parameter("lane_spacing").value:.0f} m')

    def _build_grid(self) -> list:
        width = float(self.get_parameter('field_width').value)
        height = float(self.get_parameter('field_height').value)
        alt = float(self.get_parameter('altitude').value)
        spacing = float(self.get_parameter('lane_spacing').value)
        ox = float(self.get_parameter('origin_x').value)
        oy = float(self.get_parameter('origin_y').value)
        loiter = float(self.get_parameter('loiter_time').value)
        capture = bool(self.get_parameter('capture_image').value)

        waypoints = []
        wp_id = 0
        n_lanes = max(1, int(height / spacing) + 1)

        for i in range(n_lanes):
            y = oy + min(i * spacing, height)

            # Patrón lawnmower: las pasadas pares van de izquierda a derecha
            if i % 2 == 0:
                x_start, x_end = ox, ox + width
            else:
                x_start, x_end = ox + width, ox

            # Inicio de pasada (sin captura)
            wp_start = _make_wp(wp_id, x_start, y, alt,
                                 loiter_time=0.5, capture=False,
                                 stamp=self.get_clock().now().to_msg())
            waypoints.append(wp_start)
            wp_id += 1

            # Fin de pasada (con captura opcional)
            wp_end = _make_wp(wp_id, x_end, y, alt,
                               loiter_time=loiter, capture=capture,
                               stamp=self.get_clock().now().to_msg())
            waypoints.append(wp_end)
            wp_id += 1

        return waypoints


def _make_wp(wp_id: int, x: float, y: float, z: float,
             loiter_time: float, capture: bool, stamp) -> MissionWaypoint:
    wp = MissionWaypoint()
    wp.header.stamp = stamp
    wp.header.frame_id = 'map'
    wp.waypoint_id = wp_id
    wp.pose.header.stamp = stamp
    wp.pose.header.frame_id = 'map'
    wp.pose.pose.position.x = x
    wp.pose.pose.position.y = y
    wp.pose.pose.position.z = z
    wp.pose.pose.orientation.w = 1.0
    wp.loiter_time = loiter_time
    wp.capture_image = capture
    return wp


def main(args=None):
    rclpy.init(args=args)
    node = MissionGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
