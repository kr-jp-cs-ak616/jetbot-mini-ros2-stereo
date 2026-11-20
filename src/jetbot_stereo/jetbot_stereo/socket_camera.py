"""Publish JetPack JPEG streams inside an existing ROS container."""
import copy
from pathlib import Path
import socket
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage
from .calibration import load_calibration, validate_pair
from .protocol import recv_frame


class SocketCamera(Node):
    def __init__(self):
        super().__init__('jetbot_stereo_camera')
        port = self.declare_parameter('port', 8765).value
        directory = self.declare_parameter('calibration_dir', '').value
        width = self.declare_parameter('width', 640).value
        height = self.declare_parameter('height', 480).value
        if not directory:
            raise ValueError('calibration_dir is required')
        calibrations = [load_calibration(Path(directory)/(side+'.yaml'), width, height)
                        for side in ('left', 'right')]
        baseline = validate_pair(*calibrations)
        self.infos, self.image_pubs, self.info_pubs = [], [], []
        for side, data in zip(('left', 'right'), calibrations):
            info = CameraInfo(width=width, height=height,
                              distortion_model=data['distortion_model'])
            info.header.frame_id = f'stereo_{side}_optical_frame'
            for attr, key in [('k','camera_matrix'), ('d','distortion_coefficients'),
                              ('r','rectification_matrix'), ('p','projection_matrix')]:
                setattr(info, attr, [float(v) for v in data[key]['data']])
            self.infos.append(info)
            self.image_pubs.append(self.create_publisher(
                CompressedImage, f'/stereo/{side}/image_raw/compressed', qos_profile_sensor_data))
            self.info_pubs.append(self.create_publisher(
                CameraInfo, f'/stereo/{side}/camera_info', qos_profile_sensor_data))
        self.counts = [0, 0]
        self.previous = [0, 0]
        self.stop = threading.Event()
        self.connection = None
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('127.0.0.1', port))
        self.server.listen(1)
        self.server.settimeout(1)
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()
        self.create_timer(5, self.report)
        self.get_logger().info(f'Camera receiver on 127.0.0.1:{port}; baseline {baseline*1000:.2f} mm')

    def receive(self):
        while not self.stop.is_set():
            try:
                self.connection, _ = self.server.accept()
                self.connection.settimeout(3)
                self.get_logger().info('JetPack capture connected')
                while not self.stop.is_set():
                    sensor, stamp, _, jpeg = recv_frame(self.connection)
                    msg = CompressedImage()
                    msg.header.frame_id = self.infos[sensor].header.frame_id
                    msg.header.stamp.sec = stamp // 1000000000
                    msg.header.stamp.nanosec = stamp % 1000000000
                    msg.format = 'bgr8; jpeg compressed bgr8'
                    msg.data = jpeg
                    info = copy.deepcopy(self.infos[sensor])
                    info.header = msg.header
                    self.info_pubs[sensor].publish(info)
                    self.image_pubs[sensor].publish(msg)
                    self.counts[sensor] += 1
            except socket.timeout:
                pass
            except (EOFError, OSError, ValueError) as exc:
                if not self.stop.is_set():
                    self.get_logger().warning(str(exc))
            finally:
                if self.connection is not None:
                    self.connection.close()
                    self.connection = None

    def report(self):
        rates = [(a-b)/5 for a,b in zip(self.counts, self.previous)]
        self.previous = list(self.counts)
        self.get_logger().info(f'Published left/right: {self.counts}; rates: {rates[0]:.1f}/{rates[1]:.1f} fps')

    def destroy_node(self):
        self.stop.set()
        self.server.close()
        connection = self.connection
        if connection is not None:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        self.thread.join(timeout=4)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SocketCamera()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
