"""Laptop-side JPEG decoding and rectification, preserving source timestamps."""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class Decoder(Node):
    def __init__(self):
        super().__init__('jetbot_stereo_decode')
        self.maps = {}
        self.signatures = {}
        self.pubs = {}
        self.counts = [0,0]
        for side in ('left','right'):
            self.pubs[side] = [self.create_publisher(Image,f'/stereo/{side}/'+topic,
                               qos_profile_sensor_data) for topic in ('image_raw','image_rect')]
            self.create_subscription(CameraInfo,f'/stereo/{side}/camera_info',
                                     lambda msg,s=side:self.info(s,msg),qos_profile_sensor_data)
            self.create_subscription(CompressedImage,f'/stereo/{side}/image_raw/compressed',
                                     lambda msg,s=side:self.decode(s,msg),qos_profile_sensor_data)
        self.create_timer(5,lambda:self.get_logger().info(f'Decoded left/right: {self.counts}'))

    def info(self,side,msg):
        signature=(msg.width,msg.height,tuple(msg.k),tuple(msg.d),tuple(msg.r),tuple(msg.p))
        if self.signatures.get(side)==signature:
            return
        self.maps[side]=cv2.initUndistortRectifyMap(
            np.array(msg.k).reshape(3,3),np.array(msg.d),np.array(msg.r).reshape(3,3),
            np.array(msg.p).reshape(3,4),(msg.width,msg.height),cv2.CV_32FC1)
        self.signatures[side]=signature

    def decode(self,side,msg):
        frame=cv2.imdecode(np.frombuffer(msg.data,dtype=np.uint8),cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning(f'Invalid {side} JPEG')
            return
        images=[frame]
        if side in self.maps:
            images.append(cv2.remap(frame,*self.maps[side],cv2.INTER_LINEAR))
        for publisher,image in zip(self.pubs[side],images):
            out=Image()
            out.header=msg.header
            out.height,out.width=image.shape[:2]
            out.encoding='bgr8'
            out.step=out.width*3
            out.data=image.tobytes()
            publisher.publish(out)
        self.counts[0 if side=='left' else 1]+=1


def main(args=None):
    rclpy.init(args=args)
    node=Decoder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
