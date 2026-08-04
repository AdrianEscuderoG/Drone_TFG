#!/usr/bin/env python3
"""
image_receiver_node.py

Recibe frames JPEG por UDP desde el ESP32 master (puerto 14552), decodifica
cada frame a escala de grises, corrige el timestamp de hardware con un offset
por cámara (método NTP-simplificado), y publica en /cam0/image_raw y
/cam1/image_raw.

Responsabilidad única de este nodo: UDP -> decodificar -> corregir tiempo ->
publicar. NO hace emparejamiento estéreo cam0/cam1 -- esa sincronización la
resuelve OpenVINS internamente vía message_filters::ApproximateTimeSynchronizer
sobre los timestamps ya corregidos que publica este nodo.

Formato del paquete UDP (17 bytes de cabecera + payload JPEG):
    [0:2]   magic bytes 0xCA 0xFE
    [2]     cam_id (0 o 1)
    [3:7]   frame_num, big-endian uint32 (contador independiente por cámara)
    [7:9]   frame_size, big-endian uint16 (tamaño del payload JPEG)
    [9:17]  timestamp_us, big-endian uint64 (esp_timer_get_time() del slave,
            NO comparable entre cam0 y cam1 sin el offset por cámara)
"""

import socket
import struct
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

HEADER_SIZE = 21
MAGIC_BYTES = b"\xCA\xFE"
UDP_RECV_BUFSIZE = 65535

# Número de muestras candidatas antes de congelar el offset por cámara.
# Ver razonamiento completo en el resumen de diseño: el arranque del sistema
# es el momento de mayor jitter de red, así que no usamos el primer frame;
# recogemos varias muestras y nos quedamos con el offset MÍNIMO observado,
# porque la latencia de red nunca resta tiempo, solo lo añade como error.
CALIBRATION_SAMPLES = 20


class ImageReceiverNode(Node):
    def __init__(self):
        super().__init__("image_receiver_node")

        self.declare_parameter("udp_bind_address", "0.0.0.0")
        self.declare_parameter("udp_port", 14552)

        bind_address = self.get_parameter("udp_bind_address").value
        udp_port = self.get_parameter("udp_port").value

        self.bridge = CvBridge()

        # Estado de calibración de offset, uno por cámara.
        self._offset_candidates = {0: [], 1: []}
        self._offset_fixed = {0: None, 1: None}

        # QoS best-effort: coherente con el resto del stack (ver nota en
        # /mavros/local_position/odom) y necesario para que calib_recorder_node
        # pueda suscribirse con el mismo perfil sin incompatibilidad de QoS.
        self._pub_cam0 = self.create_publisher(
            Image, "/cam0/image_raw", qos_profile_sensor_data
        )
        self._pub_cam1 = self.create_publisher(
            Image, "/cam1/image_raw", qos_profile_sensor_data
        )
        self._publishers_by_cam = {0: self._pub_cam0, 1: self._pub_cam1}

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_address, udp_port))
        # Timeout corto para poder comprobar periódicamente el flag de parada
        # sin bloquear el hilo de recepción indefinidamente.
        self._sock.settimeout(1.0)

        self._running = True
        self._recv_thread = threading.Thread(
            target=self._receive_loop, daemon=True
        )
        self._recv_thread.start()

        self.get_logger().info(
            f"image_receiver_node escuchando en {bind_address}:{udp_port}"
        )

    # ------------------------------------------------------------------
    # Parseo de cabecera (formato ya validado y estable, no tocar sin razón)
    # ------------------------------------------------------------------
    @staticmethod
    def _parse_header(data: bytes):
        if len(data) < HEADER_SIZE or data[0:2] != MAGIC_BYTES:
            return None
        cam_id = data[2]
        frame_num = struct.unpack(">I", data[3:7])[0]
        frame_size = struct.unpack(">H", data[7:9])[0]
        timestamp_us = struct.unpack(">Q", data[9:17])[0]
        cycle_id = struct.unpack(">I", data[17:21])[0]   # NUEVO
        return {
            "cam_id": cam_id,
            "frame_num": frame_num,
            "frame_size": frame_size,
            "timestamp_us": timestamp_us,
            "cycle_id": cycle_id,   # NUEVO
        }

    # ------------------------------------------------------------------
    # Offset NTP-simplificado por cámara
    # ------------------------------------------------------------------
    def _update_offset(self, cam_id: int, ts_hw_us: int, t_ros_arrival: float):
        if self._offset_fixed[cam_id] is not None:
            return
        candidate = t_ros_arrival - (ts_hw_us / 1e6)
        self._offset_candidates[cam_id].append(candidate)
        if len(self._offset_candidates[cam_id]) >= CALIBRATION_SAMPLES:
            self._offset_fixed[cam_id] = min(self._offset_candidates[cam_id])
            self.get_logger().info(
                f"Offset fijado para cam{cam_id}: "
                f"{self._offset_fixed[cam_id]:.6f} s "
                f"(tras {CALIBRATION_SAMPLES} muestras)"
            )

    def _to_ros_stamp_sec(self, cam_id: int, ts_hw_us: int):
        offset = self._offset_fixed[cam_id]
        if offset is None and self._offset_candidates[cam_id]:
            # Provisional mientras se completa la calibración de offset.
            offset = self._offset_candidates[cam_id][-1]
        if offset is None:
            return None
        return (ts_hw_us / 1e6) + offset

    # ------------------------------------------------------------------
    # Bucle de recepción UDP (hilo aparte, ver razonamiento en docstring)
    # ------------------------------------------------------------------
    def _receive_loop(self):
        while self._running and rclpy.ok():
            try:
                data, _addr = self._sock.recvfrom(UDP_RECV_BUFSIZE)
            except socket.timeout:
                continue
            except OSError:
                # Socket cerrado durante el shutdown.
                break

            header = self._parse_header(data[:HEADER_SIZE])
            if header is None:
                continue

            cam_id = header["cam_id"]
            if cam_id not in self._publishers_by_cam:
                self.get_logger().warn(
                    f"cam_id desconocido recibido: {cam_id}", throttle_duration_sec=5.0
                )
                continue

            payload = data[HEADER_SIZE:]
            if len(payload) != header["frame_size"]:
                self.get_logger().warn(
                    f"Tamaño de payload no coincide (cam{cam_id}, "
                    f"cycle={header['cycle_id']}): "
                    f"esperado {header['frame_size']}, recibido {len(payload)}",
                    throttle_duration_sec=5.0,
                )
                continue

            t_arrival = self.get_clock().now().nanoseconds / 1e9
            self._update_offset(cam_id, header["timestamp_us"], t_arrival)
            stamp_sec = self._to_ros_stamp_sec(cam_id, header["timestamp_us"])
            if stamp_sec is None:
                # Aún no hay ni siquiera un offset provisional para esta cámara.
                continue

            img = cv2.imdecode(
                np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_GRAYSCALE
            )
            if img is None:
                self.get_logger().warn(
                    f"Frame JPEG corrupto descartado (cam{cam_id}, "
                    f"cycle={header['cycle_id']}, frame_num={header['frame_num']})",
                    throttle_duration_sec=5.0,
                )
                continue

            msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
            sec = int(stamp_sec)
            nanosec = int((stamp_sec - sec) * 1e9)
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
            msg.header.frame_id = f"cam{cam_id}:{header['cycle_id']}"

            self._publishers_by_cam[cam_id].publish(msg)

    def destroy_node(self):
        self._running = False
        try:
            self._sock.close()
        except OSError:
            pass
        self._recv_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()