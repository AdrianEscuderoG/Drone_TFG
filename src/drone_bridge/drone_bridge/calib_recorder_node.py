#!/usr/bin/env python3
"""
calib_recorder_node.py

Se suscribe a /cam0/image_raw, /cam1/image_raw y al topic de IMU cruda, y
vuelca cada mensaje a disco en formato EuRoC MAV, tal como lo espera Basalt
para la calibración cámara-IMU:

    <output_dir>/mav0/cam0/data/<timestamp_ns>.png
    <output_dir>/mav0/cam0/data.csv
    <output_dir>/mav0/cam1/data/<timestamp_ns>.png
    <output_dir>/mav0/cam1/data.csv
    <output_dir>/mav0/imu0/data.csv

Responsabilidad única de este nodo: escribir a disco lo que llega, sin
sincronizar activamente cam0/cam1/IMU entre sí -- cada mensaje se graba con
su propio timestamp (ya corregido por image_receiver_node en el caso de las
cámaras). Empieza a grabar en cuanto arranca; no hay servicio de start/stop,
así que arráncalo solo después de confirmar que todos los publishers (MAVROS,
image_receiver_node) ya están activos, para no perder los primeros frames.
"""

import csv
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu


def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class CalibRecorderNode(Node):
    def __init__(self):
        super().__init__("calib_recorder_node")

        self.declare_parameter("output_dir", "~/basalt_calib_data/session")
        self.declare_parameter("cam0_topic", "/cam0/image_raw")
        self.declare_parameter("cam1_topic", "/cam1/image_raw")
        self.declare_parameter("imu_topic", "/mavros/imu/data_raw")

        output_dir = os.path.expanduser(self.get_parameter("output_dir").value)
        cam0_topic = self.get_parameter("cam0_topic").value
        cam1_topic = self.get_parameter("cam1_topic").value
        imu_topic = self.get_parameter("imu_topic").value

        self._bridge = CvBridge()

        self._mav0_dir = os.path.join(output_dir, "mav0")
        self._cam0_data_dir = os.path.join(self._mav0_dir, "cam0", "data")
        self._cam1_data_dir = os.path.join(self._mav0_dir, "cam1", "data")
        self._imu0_dir = os.path.join(self._mav0_dir, "imu0")
        for d in (self._cam0_data_dir, self._cam1_data_dir, self._imu0_dir):
            os.makedirs(d, exist_ok=True)

        self._cam0_csv = open(
            os.path.join(self._mav0_dir, "cam0", "data.csv"), "w", newline=""
        )
        self._cam1_csv = open(
            os.path.join(self._mav0_dir, "cam1", "data.csv"), "w", newline=""
        )
        self._imu0_csv = open(
            os.path.join(self._imu0_dir, "data.csv"), "w", newline=""
        )

        self._cam0_writer = csv.writer(self._cam0_csv)
        self._cam1_writer = csv.writer(self._cam1_csv)
        self._imu0_writer = csv.writer(self._imu0_csv)

        self._cam0_writer.writerow(["#timestamp [ns]", "filename"])
        self._cam1_writer.writerow(["#timestamp [ns]", "filename"])
        self._imu0_writer.writerow(
            [
                "#timestamp [ns]",
                "w_RS_S_x [rad s^-1]",
                "w_RS_S_y [rad s^-1]",
                "w_RS_S_z [rad s^-1]",
                "a_RS_S_x [m s^-2]",
                "a_RS_S_y [m s^-2]",
                "a_RS_S_z [m s^-2]",
            ]
        )

        self._cam0_count = 0
        self._cam1_count = 0
        self._imu_count = 0

        self.create_subscription(
            Image, cam0_topic, self._make_image_callback(0), qos_profile_sensor_data
        )
        self.create_subscription(
            Image, cam1_topic, self._make_image_callback(1), qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, imu_topic, self._imu_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"calib_recorder_node grabando en: {output_dir}\n"
            f"  cam0: {cam0_topic}\n  cam1: {cam1_topic}\n  imu: {imu_topic}"
        )

        # Log periódico de progreso -- útil para verificar en vivo que la
        # grabación cubre los tres ejes de rotación y las esquinas del FOV,
        # no solo el centro (criterio ya acordado para que Basalt no dé
        # métricas de reproyección pobres y haya que regrabar).
        self._status_timer = self.create_timer(5.0, self._log_status)

    def _make_image_callback(self, cam_id: int):
        data_dir = self._cam0_data_dir if cam_id == 0 else self._cam1_data_dir
        writer = self._cam0_writer if cam_id == 0 else self._cam1_writer

        def callback(msg: Image):
            ts_ns = stamp_to_ns(msg.header.stamp)
            filename = f"{ts_ns}.png"
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            cv2.imwrite(os.path.join(data_dir, filename), cv_image)
            writer.writerow([ts_ns, filename])
            if cam_id == 0:
                self._cam0_csv.flush()
                self._cam0_count += 1
            else:
                self._cam1_csv.flush()
                self._cam1_count += 1

        return callback

    def _imu_callback(self, msg: Imu):
        ts_ns = stamp_to_ns(msg.header.stamp)
        self._imu0_writer.writerow(
            [
                ts_ns,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]
        )
        self._imu0_csv.flush()
        self._imu_count += 1

    def _log_status(self):
        self.get_logger().info(
            f"Grabados hasta ahora -> cam0: {self._cam0_count}, "
            f"cam1: {self._cam1_count}, imu0: {self._imu_count}"
        )

    def destroy_node(self):
        for f in (self._cam0_csv, self._cam1_csv, self._imu0_csv):
            try:
                f.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()