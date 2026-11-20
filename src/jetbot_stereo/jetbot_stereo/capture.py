"""CSI capture through JetPack Argus. Arrival stamps are not hardware sync."""
import copy
from pathlib import Path
import threading
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from .calibration import load_calibration


class StereoCapture(Node):
    def __init__(self):
        super().__init__('csi_stereo')
        defaults = dict(width=640, height=480, fps=15, left_sensor_id=0,
                        right_sensor_id=1, calibration_dir='', sensor_mode=-1,
                        flip_method=0)
        self.cfg = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if self.cfg['left_sensor_id'] == self.cfg['right_sensor_id']:
            raise ValueError('Left and right sensor IDs must differ')
        if min(self.cfg[k] for k in ('width', 'height', 'fps')) <= 0:
            raise ValueError('width, height and fps must be positive')
        if 'GStreamer:                   YES' not in cv2.getBuildInformation():
            # Spacing differs across OpenCV builds.
            if not any('GStreamer' in line and 'YES' in line
                       for line in cv2.getBuildInformation().splitlines()):
                raise RuntimeError('OpenCV requires GStreamer support (use apt python3-opencv)')
        self.stop = threading.Event()
        self.caps, self.threads = [], []
        try:
            for side in ('left', 'right'):
                info = CameraInfo(width=self.cfg['width'], height=self.cfg['height'])
                if self.cfg['calibration_dir']:
                    d = load_calibration(Path(self.cfg['calibration_dir']) / (side + '.yaml'),
                                         self.cfg['width'], self.cfg['height'])
                    info.distortion_model = d['distortion_model']
                    info.d = [float(v) for v in d['distortion_coefficients']['data']]
                    for attr, key in [('k', 'camera_matrix'), ('r', 'rectification_matrix'),
                                      ('p', 'projection_matrix')]:
                        setattr(info, attr, [float(v) for v in d[key]['data']])
                sensor_id = self.cfg[side + '_sensor_id']
                pipeline = (
                    f'nvarguscamerasrc sensor-id={sensor_id} sensor-mode={self.cfg["sensor_mode"]} ! '
                    f'video/x-raw(memory:NVMM),width={self.cfg["width"]},'
                    f'height={self.cfg["height"]},framerate={self.cfg["fps"]}/1,format=NV12 ! '
                    f'nvvidconv flip-method={self.cfg["flip_method"]} ! '
                    'video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! '
                    'appsink drop=true max-buffers=1 sync=false')
                cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                self.caps.append(cap)
                if not cap.isOpened():
                    raise RuntimeError(f'Cannot open CSI {side}, sensor-id={sensor_id}; check sensor mode and Argus')
                image_pub = self.create_publisher(Image, f'/stereo/{side}/image_raw', qos_profile_sensor_data)
                info_pub = self.create_publisher(CameraInfo, f'/stereo/{side}/camera_info', qos_profile_sensor_data)
                thread = threading.Thread(target=self.capture, args=(side, cap, info, image_pub, info_pub), daemon=True)
                self.threads.append(thread)
            for thread in self.threads:
                thread.start()
        except Exception:
            for cap in self.caps:
                cap.release()
            raise

    def capture(self, side, cap, info, image_pub, info_pub):
        bridge = CvBridge()
        try:
            while rclpy.ok() and not self.stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError(f'{side} CSI frame read failed')
                image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                image.header.stamp = self.get_clock().now().to_msg()
                image.header.frame_id = f'stereo_{side}_optical_frame'
                camera_info = copy.deepcopy(info)
                camera_info.header = image.header
                image_pub.publish(image)
                info_pub.publish(camera_info)
        except Exception as exc:
            self.get_logger().fatal(str(exc))
            self.stop.set()
            self.context.try_shutdown()

    def close(self):
        self.stop.set()
        for thread in self.threads:
            thread.join(timeout=2)
        for cap in self.caps:
            cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StereoCapture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
