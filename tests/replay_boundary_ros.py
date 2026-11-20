import cv2,json,time,subprocess,signal,sys,yaml,sqlite3,os
from pathlib import Path
import numpy as np,rclpy
from sensor_msgs.msg import Image,CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from rclpy.qos import qos_profile_sensor_data
assert os.environ.get('ROS_DOMAIN_ID')=='202' and os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST','Run this replay only in isolated ROS domain 202/LOCALHOST'
root=Path.cwd();out=root/'log/boundary-real-odom-map.json'
if out.exists():out=out.with_name('boundary-replay-map-'+str(time.time_ns())+'.json')
log=(root/'log/boundary-real-odom-replay.log').open('w')
p=subprocess.Popen(['ros2','launch','jetbot_stereo','partition.launch.py','calibration_dir:='+str(root/'calibration'),'map_file:='+str(out)],stdout=log,stderr=subprocess.STDOUT)
rclpy.init();n=rclpy.create_node('partition_replay_verifier');pubs={};frames={};infos={};observed={}
for s in ['left','right']:
 connection=sqlite3.connect((root/'maps/jetbot-20261117-143146.db').as_uri()+'?mode=ro',uri=True)
 blob=connection.execute('select '+('image' if s=='left' else 'depth')+' from Data where id=149').fetchone()[0];connection.close()
 image=cv2.imdecode(np.frombuffer(blob,np.uint8),cv2.IMREAD_COLOR)
 msg=Image();msg.height=480;msg.width=640;msg.encoding='bgr8';msg.step=1920;msg.data=image.tobytes();msg.header.frame_id='stereo_'+s+'_optical_frame';frames[s]=msg
 d=yaml.safe_load(open('calibration/'+s+'.yaml'));info=CameraInfo(width=640,height=480,distortion_model='plumb_bob');info.k=d['camera_matrix']['data'];info.d=d['distortion_coefficients']['data'];info.r=d['rectification_matrix']['data'];info.p=d['projection_matrix']['data'];infos[s]=info
 pubs[s]=[n.create_publisher(Image,'/stereo/'+s+'/image_rect',qos_profile_sensor_data),n.create_publisher(CameraInfo,'/stereo/'+s+'/camera_info',qos_profile_sensor_data)]

marker_ids=set();marker_updates=[];clears=[]
def marker_frame(m):
 ids={x.id for x in m.markers if x.ns=='wall_hypotheses' and x.action==0}
 if ids:marker_updates.append(sorted(ids));marker_ids.update(ids)
 clears.extend(x for x in m.markers if x.action==3)
n.create_subscription(MarkerArray,'/partition/wall_markers',marker_frame,1)
n.create_subscription(String,'/partition/walls',lambda m:observed.__setitem__('walls',json.loads(m.data)),1)
def masked(m):
 a=np.frombuffer(m.data,np.uint8).reshape(480,640,3)
 observed['background_zero']=bool(np.all(a[50:120,350:600]==0));observed['wall_kept']=bool(a[300,400].any())
n.create_subscription(Image,'/partition/left/image_rect',masked,qos_profile_sensor_data)
end=time.monotonic()+12
try:
 while time.monotonic()<end:
  stamp=n.get_clock().now().to_msg()

  for s in frames:
   frames[s].header.stamp=stamp;infos[s].header=frames[s].header;pubs[s][1].publish(infos[s]);pubs[s][0].publish(frames[s])
  until=time.monotonic()+1/15
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.005)
finally:
 p.send_signal(signal.SIGINT);p.wait(timeout=10);log.close();n.destroy_node();rclpy.shutdown()
assert observed.get('background_zero') and observed.get('wall_kept'),observed
report=json.loads(out.read_text());assert len(report['wall_cells']['occupied_cells'])>0
assert report['images_stored'] is False and report['background_stored'] is False
assert len(report['wall_segments'])==2 and len(marker_ids)==2,(report,marker_ids)
assert len(clears)<=1
assert marker_updates and all(x==marker_updates[-1] for x in marker_updates[-3:])
result={'isolated_ros_domain':202,'source':'saved rectified DB frame 149','test_pose':'recorded stationary images with preprocessed transient stereo odometry and masked map inputs; actual robot motion remains untested','walls':len(observed['walls']['walls']),'stored_cells':len(report['wall_cells']['occupied_cells']),'background_zero':observed['background_zero'],'stable_wall_ids':sorted(marker_ids),'full_marker_clears':len(clears),'map_bytes':out.stat().st_size,'map_file':str(out)}
(root/'log/boundary-real-odom-validation.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
