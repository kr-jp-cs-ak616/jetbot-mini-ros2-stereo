"""Keep the saved wall map visible independently of live camera tracking."""
import json,math
from pathlib import Path
import numpy as np


def wall_map_center(segments):
    points=[]
    for s in segments[:256]:
        p=np.asarray(s.get('endpoints_xy_m',[]),float)
        h=float(s.get('height_m',0));z=float(s.get('floor_z_m',0))
        if p.shape!=(2,2) or not np.isfinite(p).all() or not math.isfinite(h+z):continue
        for x,y in p:points.extend([[x,y,z],[x,y,z+h]])
    if not points:return [0.,0.,0.]
    points=np.array(points);return ((points.min(axis=0)+points.max(axis=0))/2).tolist()


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
    class MapView(Node):
        def __init__(self):
            super().__init__('partition_map_view_frame')
            self.owner_pid=int(self.declare_parameter('owner_pid',0).value)
            self.file=Path(self.declare_parameter('map_file','').value);self.center=[0.,0.,0.]
            self.dynamic=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
            # Register the arbitrary map origin; this is not an invented robot pose.
            origin=TransformStamped();origin.header.stamp=self.get_clock().now().to_msg()
            origin.header.frame_id='boundary_map_origin';origin.child_frame_id='partition_odom';origin.transform.rotation.w=1.
            self.static.sendTransform(origin);self.create_timer(.5,self.update);self.update()
        def update(self):
            if self.owner_pid and not Path('/proc/'+str(self.owner_pid)).exists():
                rclpy.try_shutdown();return
            try:
                data=json.loads(self.file.read_text())
                if data.get('frame_id')=='partition_odom':self.center=wall_map_center(data.get('wall_segments',[]))
            except (OSError,ValueError,TypeError):pass
            frame=TransformStamped();frame.header.stamp=self.get_clock().now().to_msg()
            frame.header.frame_id='partition_odom';frame.child_frame_id='partition_map_view'
            frame.transform.translation.x,frame.transform.translation.y,frame.transform.translation.z=self.center
            frame.transform.rotation.w=1.;self.dynamic.sendTransform(frame)
    rclpy.init(args=args);n=MapView()
    try:rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
