#!/usr/bin/env python3
"""
stereo_camera_info_fix_node.py

Corrige el campo Tx de la matriz de proyección P en el CameraInfo de la
cámara derecha del par estéreo simulado.

Responsabilidad única: leer CameraInfo de la cámara derecha, corregir Tx,
republicar. NO recalcula fx/fy/cx/cy ni ningún otro campo -- se copian tal
cual del mensaje original.

Tx se calcula como -fx * baseline, leyendo fx directamente de k[0] del
mensaje entrante (no se hardcodea), de modo que el nodo sigue siendo
correcto si cambia la resolución/FOV simulado. El baseline sí es un
parámetro fijo porque depende de la geometría física de las cámaras en el
SDF, no del mensaje.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class StereoCameraInfoFixNode(Node):
    """
    Parámetros ROS 2:
      baseline (float, default 0.12):
        Distancia entre cam0 y cam1 en metros, según el SDF
        (iris_with_stereo: cam0 en Y=+0.06, cam1 en Y=-0.06).
      input_topic (str, default "/cam1/camera_info"):
        CameraInfo original de la cámara derecha, sin corregir.
      output_topic (str, default "/cam1/camera_info_corrected"):
        CameraInfo republicado con Tx corregido. Es el topic que debe
        remapearse en el YAML de RTAB-Map, no el original.
    """

    def __init__(self):
        super().__init__("stereo_camera_info_fix_node")

        self.declare_parameter("baseline", 0.12)
        self.declare_parameter("input_topic", "/cam1/camera_info")
        self.declare_parameter("output_topic", "/cam1/camera_info_corrected")

        self._baseline = self.get_parameter("baseline").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        # QoS best-effort: coherente con el resto de topics de sensores
        # de simulación (mismo criterio que image_receiver_node).
        self._pub = self.create_publisher(
            CameraInfo, output_topic, qos_profile_sensor_data
        )
        self._sub = self.create_subscription(
            CameraInfo, input_topic, self._on_camera_info, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"stereo_camera_info_fix_node iniciado — "
            f"baseline={self._baseline} m, {input_topic} -> {output_topic}"
        )

    def _on_camera_info(self, msg: CameraInfo) -> None:
        fx = msg.k[0]
        tx_corrected = -fx * self._baseline

        # p es una tupla de solo lectura en el mensaje entrante -- se
        # reconstruye como lista mutable para corregir únicamente p[3].
        p_corrected = list(msg.p)
        p_corrected[3] = tx_corrected
        msg.p = p_corrected

        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraInfoFixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()