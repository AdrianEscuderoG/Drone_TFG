#!/usr/bin/env python3
"""
camera_info_pub.py

Publica sensor_msgs/CameraInfo en un topic a partir de un fichero YAML
de calibración (formato compatible con camera_calibration de ROS).

Uso:
  ros2 run drone_slam camera_info_pub \
      --ros-args \
      -p yaml_path:=/ruta/a/calibracion.yaml \
      -p camera_info_topic:=/cam0/camera_info \
      -p image_topic:=/cam0/image_raw \
      -p rate_hz:=20.0

Parámetros:
  yaml_path           Ruta al fichero YAML de calibración
  camera_info_topic   Topic donde publicar CameraInfo
  image_topic         Topic de imagen al que sincronizarse (opcional)
                      Si se especifica, publica solo cuando llega una imagen.
                      Si está vacío, publica a rate_hz Hz de forma continua.
  rate_hz             Frecuencia de publicación (solo si image_topic está vacío)

Por qué sincronizar con la imagen:
  RTAB-Map usa un sincronizador aproximado entre image y camera_info.
  Si camera_info se publica a una frecuencia distinta o con timestamps
  diferentes, el sincronizador los descarta. Escuchar la imagen y
  republicar camera_info con el mismo timestamp garantiza la sincronización.
"""

import sys
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo, Image


RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def load_camera_info(yaml_path: str) -> CameraInfo:
    """Lee un YAML de calibración y devuelve un mensaje CameraInfo."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    msg = CameraInfo()
    msg.width  = int(data['image_width'])
    msg.height = int(data['image_height'])

    msg.distortion_model = data.get('distortion_model', 'plumb_bob')

    msg.k = [float(v) for v in data['camera_matrix']['data']]
    msg.d = [float(v) for v in data['distortion_coefficients']['data']]
    msg.r = [float(v) for v in data['rectification_matrix']['data']]
    msg.p = [float(v) for v in data['projection_matrix']['data']]

    return msg


class CameraInfoPublisher(Node):

    def __init__(self):
        super().__init__('camera_info_pub')

        self.declare_parameter('yaml_path',          '')
        self.declare_parameter('camera_info_topic',  '/cam0/camera_info')
        self.declare_parameter('image_topic',        '/cam0/image_raw')
        self.declare_parameter('rate_hz',            20.0)

        yaml_path         = self.get_parameter('yaml_path').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        image_topic       = self.get_parameter('image_topic').value
        rate_hz           = self.get_parameter('rate_hz').value

        if not yaml_path:
            self.get_logger().error('Parámetro yaml_path no especificado.')
            sys.exit(1)

        try:
            self._camera_info = load_camera_info(yaml_path)
            self.get_logger().info(f'Calibración cargada desde: {yaml_path}')
            self.get_logger().info(
                f'Resolución: {self._camera_info.width}x{self._camera_info.height}')
        except Exception as e:
            self.get_logger().error(f'Error leyendo YAML: {e}')
            sys.exit(1)

        self._pub = self.create_publisher(
            CameraInfo, camera_info_topic, RELIABLE_QOS)

        if image_topic:
            # Modo sincronizado: publica camera_info cada vez que llega
            # una imagen, copiando su timestamp y frame_id.
            # Esto garantiza que el sincronizador de RTAB-Map los empareje.
            self.get_logger().info(
                f'Modo sincronizado con imagen: {image_topic}')
            self.create_subscription(
                Image, image_topic, self._on_image, RELIABLE_QOS)
        else:
            # Modo timer: publica a frecuencia fija sin sincronizar.
            self.get_logger().info(
                f'Modo timer a {rate_hz} Hz')
            self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f'Publicando CameraInfo en: {camera_info_topic}')

    def _on_image(self, img_msg: Image) -> None:
        """Callback de imagen: publica camera_info con el mismo timestamp."""
        msg = CameraInfo()
        msg = self._camera_info
        # Copiar header de la imagen para sincronización exacta
        msg.header.stamp    = img_msg.header.stamp
        msg.header.frame_id = img_msg.header.frame_id
        self._pub.publish(msg)

    def _on_timer(self) -> None:
        """Callback de timer: publica con timestamp actual."""
        self._camera_info.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._camera_info)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
