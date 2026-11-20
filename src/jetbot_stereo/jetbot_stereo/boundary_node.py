"""Preprocess stereo for transient tracking; persist only bounded masked wall cells."""
import copy,json,math,time
from collections import deque
from pathlib import Path
import cv2,numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,LaserScan
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker,MarkerArray
from .calibration import load_calibration,validate_pair
from .boundary_walls import model_partitions,SparseWallMap,ObservationBuffer
from .wall_scan import (ANGLE_MIN, ANGLE_STEP, BEAM_COUNT, MIN_SCAN_BEAMS, RANGE_MIN,
                        RANGE_MAX, WallScanConfirmation, wall_ranges)
from .wall_tracks import WallTracks,RelativeHeading,PoseContinuity
from .reference_motion import ReferenceMotionGuard


def quaternion_matrix(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class BoundaryWalls(Node):
    def __init__(self):
        super().__init__('partition_boundary_mapper');cv2.setNumThreads(2)
        self.height=float(self.declare_parameter('camera_height_m',.099).value)
        self.pitch=float(self.declare_parameter('camera_pitch_deg',0.).value)
        measured_height=float(self.declare_parameter('partition_height_m',0.).value)
        if measured_height!=0. and not .015<=measured_height<=2.:
            raise ValueError('Invalid measured partition height')
        self.partition_height=measured_height or None
        self.partition_pitch_tolerance=float(self.declare_parameter('partition_pitch_tolerance_deg',0.).value)
        if not 0.<=self.partition_pitch_tolerance<=4.:
            raise ValueError('Invalid partition pitch tolerance')
        self.min_panel_luma=float(self.declare_parameter('min_panel_luma',75.).value)
        if not 0.<=self.min_panel_luma<=255.:
            raise ValueError('Invalid minimum panel brightness')
        self.mask_tracking=bool(self.declare_parameter('tracking_masked_only',False).value)
        directory=Path(self.declare_parameter('calibration_dir','').value)
        self.localization_only=self.declare_parameter('localization_only',False).value
        guard_enabled=self.declare_parameter('guard_reference_odometry',False).value
        self.output=Path(self.declare_parameter('map_file','').value)
        if not .02<=self.height<=2 or abs(self.pitch)>30:raise ValueError('Invalid camera height/pitch')
        if not self.localization_only:
            if self.output == Path('.') or self.output.exists():
                raise ValueError('Choose a new map_file; existing maps are preserved')
            self.output.parent.mkdir(parents=True,exist_ok=True)
        self.cal=[load_calibration(directory/(s+'.yaml'),640,480) for s in ('left','right')];validate_pair(*self.cal)
        self.frames={s:deque(maxlen=3) for s in ('left','right')};self.infos={};self.last_stamp=None
        self.recent_poses=deque(maxlen=30)
        self.pending=ObservationBuffer();self.last_integrated=None;self.last_integration_received=0.
        self.pose=None;self.pose_valid=False;self.wallmap=SparseWallMap();self.wall_tracks=WallTracks();self.segments={};self.last_written=0.;self.frozen=False
        self.heading=RelativeHeading();self.heading_status={};self.pose_continuity=PoseContinuity();self.pose_hold_reason='waiting_for_odometry';self.published_wall_ids=set();self.clear_markers=True
        self.images={};self.info_pubs={};self.tracking_images={}
        for s in self.frames:
            self.images[s]=self.create_publisher(Image,'/partition/'+s+'/image_rect',qos_profile_sensor_data)
            self.tracking_images[s]=self.create_publisher(Image,'/partition/tracking/'+s+'/image_rect',qos_profile_sensor_data)
            self.info_pubs[s]=self.create_publisher(CameraInfo,'/partition/'+s+'/camera_info',qos_profile_sensor_data)
            self.create_subscription(Image,'/stereo/'+s+'/image_rect',lambda m,s=s:self.frames[s].append(m),qos_profile_sensor_data)
            self.create_subscription(CameraInfo,'/stereo/'+s+'/camera_info',lambda m,s=s:self.infos.__setitem__(s,m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/partition/odom',self.odometry,qos_profile_sensor_data)
        self.status=self.create_publisher(String,'/partition/walls',1)
        self.markers=self.create_publisher(MarkerArray,'/partition/wall_markers',1)
        self.debug=self.create_publisher(Image,'/partition/preprocessed',qos_profile_sensor_data)
        self.scan = None
        self.motion_guard = None
        if self.localization_only:
            self.scan = self.create_publisher(LaserScan, '/partition/wall_scan', qos_profile_sensor_data)
            self.scan_confirmation = WallScanConfirmation()
            self.scan_state = 'waiting_for_odometry'
            if guard_enabled:
                self.motion_guard = ReferenceMotionGuard()
                self.checked_odom = self.create_publisher(Odometry, '/partition/odom_checked', qos_profile_sensor_data)
                self.checked_tf = TransformBroadcaster(self)
        self.create_timer(.2,self.process)
        self.get_logger().info('Experimental boundary mapping: preprocessed transient tracking; compact wall JSON, no image DB. Pitch must be verified for metric accuracy.')
    @staticmethod
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9
    def odometry(self,m):
        q=m.pose.pose.orientation;norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        self.pose_valid=bool(.95<norm<1.05 and 0<=m.pose.covariance[0]<.25)
        self.pose=m;self.pose_hold_reason='' if self.pose_valid else 'visual_tracking_invalid'
        if self.localization_only and not self.pose_valid:
            self.clear_scan_confirmation()
        if self.pose_valid:
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            t=m.pose.pose.position
            self.pose_valid=self.pose_continuity.accept(self.stamp(m),[t.x,t.y,t.z],yaw)
            if not self.pose_valid:
                self.pose_hold_reason='discontinuous_visual_pose; wall updates held'
                if self.localization_only:
                    self.clear_scan_confirmation()
                return
            self.heading_status=self.heading.update(yaw)
            self.recent_poses.append(m)
            observation=self.pending.match(self.stamp(m), tolerance=.02 if self.localization_only else .025)
            if observation is not None and (self.last_integrated is None or observation[0]>self.last_integrated):
                if self.localization_only:
                    self.publish_wall_scan(observation, m)
                else:
                    self.integrate(observation[1],m)
                self.last_integrated=observation[0];self.last_integration_received=time.monotonic()
    @staticmethod
    def pixels(m):
        if m.encoding not in ('bgr8','rgb8'):raise ValueError('Expected 8-bit color rectified input')
        a=np.frombuffer(m.data,np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
        return cv2.cvtColor(a,cv2.COLOR_RGB2BGR) if m.encoding=='rgb8' else a
    @staticmethod
    def image(data,header):
        msg=Image();msg.header=header;msg.height,msg.width=data.shape[:2];msg.encoding='bgr8';msg.step=msg.width*3;msg.data=data.tobytes();return msg
    def process(self):
        if self.frozen or not self.frames['left'] or len(self.infos)!=2:return
        left=self.frames['left'][-1]
        if self.stamp(left)==self.last_stamp:return
        right=None
        if not self.localization_only:
            if not self.frames['right']:return
            right=min(self.frames['right'],key=lambda m:abs(self.stamp(m)-self.stamp(left)))
            if abs(self.stamp(left)-self.stamp(right))>.02:return
        self.last_stamp=self.stamp(left);models=[]
        for s,m,c in zip(('left','right'),(left,right),self.cal):
            if not np.allclose(self.infos[s].p,c['projection_matrix']['data']):
                self.get_logger().error('CameraInfo differs from configured calibration');return
            model=model_partitions(self.pixels(m),c['projection_matrix']['data'],c['rectification_matrix']['data'],
                                   self.height,self.pitch,partition_height_m=self.partition_height,
                                   partition_pitch_tolerance_deg=self.partition_pitch_tolerance,
                                   min_panel_luma=self.min_panel_luma)
            models.append(model)
            if self.localization_only:
                break
            tracking=cv2.cvtColor(model['preprocessing']['contrast'],cv2.COLOR_GRAY2BGR)
            if self.mask_tracking:tracking[~model['keep_mask']]=0
            self.tracking_images[s].publish(self.image(tracking,m.header))
            self.images[s].publish(self.image(model['masked_rgb'],m.header))
            info=copy.deepcopy(self.infos[s]);info.header=m.header;self.info_pubs[s].publish(info)
        self.debug.publish(self.image(models[0]['preview'],left.header))
        self.pending.add(self.stamp(left),models[0]['walls'])
        if self.localization_only and self.pose_valid:
            # Direct image odometry can finish before the slower wall detection.
            for pose in reversed(self.recent_poses):
                observation = self.pending.match(self.stamp(pose), tolerance=.02)
                if observation is not None and (self.last_integrated is None or observation[0]>self.last_integrated):
                    self.publish_wall_scan(observation, pose)
                    self.last_integrated = observation[0]
                    self.last_integration_received = time.monotonic()
                    break
        pose_ok=self.pose_valid and time.monotonic()-self.last_integration_received<1.5
        report={'walls':models[0]['walls'],'pose_valid_for_map':bool(pose_ok),'stored_cells':len(self.wallmap.cells),'background_stored':False,'metric_verified':False,'pitch_assumed_deg':self.pitch,'visual_heading':self.heading_status,'heading_valid':self.pose_valid,'map_hold_reason':self.pose_hold_reason,'wall_count':len(self.segments),'wall_ids':sorted(self.segments)}
        if self.localization_only:
            report['wall_scan_state'] = self.scan_state if pose_ok else self.pose_hold_reason or 'waiting_for_matching_odometry'
        self.status.publish(String(data=json.dumps(report)))
        if time.monotonic()-self.last_written>1:self.save()
    def integrate(self,walls,pose):
        q=pose.pose.pose.orientation;r=quaternion_matrix(q);t=pose.pose.pose.position
        observations=[]
        for wall in walls:
            optical=np.array(wall['floor_endpoints_optical']);base=np.column_stack([optical[:,2],-optical[:,0],-optical[:,1]])
            world=base@r.T+np.array([t.x,t.y,t.z]);points=world[:,:2]
            if np.linalg.norm(points[1]-points[0])<.05:continue
            observations.append({'endpoints_xy_m':points.tolist(),'height_m':wall['height_m'],'floor_z_m':float(world[:,2].mean())})
        self.wall_tracks.observe(observations,self.stamp(pose))
        self.segments={s['id']:s for s in self.wall_tracks.confirmed()}
        # Rebuild from current associated walls: an updated wall must not leave old cells behind.
        updated=SparseWallMap()
        try:
            for segment in self.segments.values():updated.add_segment(*segment['endpoints_xy_m'])
        except ValueError as e:
            self.frozen=True;self.get_logger().error(str(e));return
        self.wallmap=updated
    def clear_scan_confirmation(self):
        self.recent_poses.clear()
        self.scan_confirmation.clear()
        self.scan_state = 'waiting_for_odometry'

    def publish_wall_scan(self, observation, pose):
        stamp,walls = observation
        p = pose.pose.pose
        rotation = quaternion_matrix(p.orientation)
        translation = np.array([p.position.x,p.position.y,p.position.z])
        confirmed = self.scan_confirmation.confirm(walls, stamp, rotation, translation)
        if self.motion_guard is not None:
            hold_reason = self.motion_guard.check(walls, confirmed, rotation, translation)
            if hold_reason is not None:
                self.scan_state = hold_reason
                return
            transform = TransformStamped(header=pose.header, child_frame_id=pose.child_frame_id)
            transform.transform.translation.x = p.position.x
            transform.transform.translation.y = p.position.y
            transform.transform.translation.z = p.position.z
            transform.transform.rotation = p.orientation
            self.checked_tf.sendTransform(transform)
            self.checked_odom.publish(pose)
        ranges = wall_ranges(confirmed)
        if sum(math.isfinite(value) for value in ranges) < MIN_SCAN_BEAMS:
            self.scan_state = 'waiting_for_consistent_wall' if walls else 'no_wall_detected'
            return
        scan = LaserScan()
        scan.header.frame_id = 'stereo_camera_link'
        scan.header.stamp = pose.header.stamp
        scan.angle_min = ANGLE_MIN
        scan.angle_increment = ANGLE_STEP
        scan.angle_max = ANGLE_MIN + (BEAM_COUNT - 1) * ANGLE_STEP
        scan.range_min = RANGE_MIN
        scan.range_max = RANGE_MAX
        scan.ranges = ranges
        self.scan.publish(scan)
        self.scan_state = 'published'

    def save(self):
        if self.localization_only:
            self.last_written = time.monotonic()
            return
        markers=MarkerArray()
        if self.clear_markers:
            clear=Marker();clear.header.frame_id='partition_odom';clear.action=Marker.DELETEALL;markers.markers.append(clear);self.clear_markers=False
        for identifier in self.published_wall_ids-set(self.segments):
            deleted=Marker();deleted.header.frame_id='partition_odom';deleted.ns='wall_hypotheses';deleted.id=identifier;deleted.action=Marker.DELETE;markers.markers.append(deleted)
        for index,segment in sorted(self.segments.items()):
            points=np.array(segment['endpoints_xy_m']);delta=points[1]-points[0];center=points.mean(axis=0)
            marker=Marker();marker.header.frame_id='partition_odom';marker.header.stamp=self.get_clock().now().to_msg()
            marker.ns='wall_hypotheses';marker.id=index;marker.type=Marker.CUBE;marker.action=Marker.ADD
            marker.pose.position.x=float(center[0]);marker.pose.position.y=float(center[1])
            marker.pose.position.z=segment['floor_z_m']+segment['height_m']/2
            yaw=math.atan2(delta[1],delta[0]);marker.pose.orientation.z=math.sin(yaw/2);marker.pose.orientation.w=math.cos(yaw/2)
            marker.scale.x=float(np.linalg.norm(delta));marker.scale.y=.02;marker.scale.z=segment['height_m']
            marker.color.r=.1;marker.color.g=.7;marker.color.b=1.;marker.color.a=.7;markers.markers.append(marker)
        self.published_wall_ids=set(self.segments)
        self.markers.publish(markers)
        report={'type':'experimental_boundary_wall_map','frame_id':'partition_odom','camera_height_m':self.height,'assumed_pitch_deg':self.pitch,'metric_verified':False,'background_stored':False,'images_stored':False,'background_tracking_temporary_only':not self.mask_tracking,'wall_association':'persistent_ids_with_geometric_matching','visual_heading':self.heading_status,'wall_segments':list(self.segments.values()),'wall_cells':self.wallmap.serialize()}
        tmp=self.output.with_suffix('.tmp');tmp.write_text(json.dumps(report)+'\n');tmp.replace(self.output);self.last_written=time.monotonic()
    def destroy_node(self):
        self.save();return super().destroy_node()


def main(args=None):
    rclpy.init(args=args);n=None
    try:n=BoundaryWalls();rclpy.spin(n)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if n is not None:n.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
