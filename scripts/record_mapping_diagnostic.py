#!/usr/bin/env python3
"""Record Nano's existing compressed stream without opening CSI cameras again."""
import argparse,json,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log

p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--seconds',type=float,default=30);p.add_argument('--raw',action='store_true',help='Capture from the local decoder')
a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
rclpy.init();n=rclpy.create_node('jetbot_mapping_diagnostic_recorder')
files={s:(a.output/(s+'.mjpeg')).open('wb') for s in ('left','right')}
events=(a.output/'events.jsonl').open('w');counts={s:0 for s in files}
def event(data):
 data['received']=time.time();events.write(json.dumps(data)+'\n')
def frame(m,side):
 stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
 data=bytes(m.data);offset=files[side].tell();files[side].write(data)
 event(dict(kind='frame',side=side,index=counts[side],stamp=stamp,offset=offset,length=len(data)))
 counts[side]+=1
def raw_frame(message, side):
 import cv2
 import numpy as np
 if message.encoding not in ('bgr8','rgb8'):
  return
 rows=np.frombuffer(message.data,np.uint8).reshape(message.height,message.step)
 image=rows[:,:message.width*3].reshape(message.height,message.width,3)
 if message.encoding=='rgb8':
  image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
 ok,jpeg=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,95])
 if ok:
  stamped=CompressedImage()
  stamped.header=message.header
  stamped.data=jpeg.tobytes()
  frame(stamped,side)
for side in files:
 if a.raw:
  n.create_subscription(Image,'/stereo/'+side+'/image_raw',lambda m,s=side:raw_frame(m,s),qos_profile_sensor_data)
 else:
  n.create_subscription(CompressedImage,'/stereo/'+side+'/image_raw/compressed',lambda m,s=side:frame(m,s),qos_profile_sensor_data)
def log(m):
 if 'rtabmap' in m.name or 'stereo_odometry' in m.name:
  event(dict(kind='log',name=m.name,level=m.level,message=m.msg,stamp=m.stamp.sec+m.stamp.nanosec/1e9))
n.create_subscription(Log,'/rosout',log,100)
n.create_subscription(Odometry,'/stereo/odom',lambda m:event(dict(kind='odom',stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,covariance=list(m.pose.covariance),position=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])),qos_profile_sensor_data)
started=time.time();end=time.monotonic()+a.seconds
print('Recording',a.output,flush=True)
try:
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
finally:
 for f in files.values():f.close()
 events.close();n.destroy_node();rclpy.shutdown()
 (a.output/'recording.json').write_text(json.dumps(dict(start=started,end=time.time(),counts=counts),indent=2))
 print(counts,flush=True)
