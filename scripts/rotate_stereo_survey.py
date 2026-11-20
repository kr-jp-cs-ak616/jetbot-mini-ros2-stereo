#!/usr/bin/env python3
"""Bounded visual-odometry yaw sweep; record original Nano JPEGs, stop on faults."""
import argparse,json,math,time,signal
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool,Trigger


class TrackingUnavailable(RuntimeError):
    pass


def angle_delta(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path);p.add_argument('--degrees',type=float,default=360)
    p.add_argument('--timeout',type=float,default=120)
    a=p.parse_args()
    if not 0<a.degrees<=360 or not 1<=a.timeout<=180:p.error('degrees 0..360, timeout 1..180')
    a.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();n=rclpy.create_node('jetbot_rotation_survey')
    from rclpy.qos import qos_profile_sensor_data
    streams={s:(a.output/(s+'.mjpeg')).open('wb') for s in ('left','right')}
    events=(a.output/'events.jsonl').open('w');counts={s:0 for s in streams}
    frame_times={s:0. for s in streams}
    state={'position':None,'origin':None,'status':None,'status_time':0,'pose_time':0,'yaw':None,'previous_yaw':None,'total':0.,'valid':False}
    def event(d):d['received']=time.time();events.write(json.dumps(d)+'\n')
    def frame(m,s):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9;b=bytes(m.data)
        off=streams[s].tell();streams[s].write(b)
        event(dict(kind='frame',side=s,index=counts[s],stamp=stamp,offset=off,length=len(b)))
        counts[s]+=1;frame_times[s]=time.monotonic()
    for s in streams:n.create_subscription(CompressedImage,'/stereo/'+s+'/image_raw/compressed',lambda m,s=s:frame(m,s),qos_profile_sensor_data)
    def status(m):state['status']=json.loads(m.data);state['status_time']=time.monotonic()
    n.create_subscription(String,'/jetbot/status',status,10)
    def pose(m):
        q=m.pose.pose.orientation;stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
        valid=(.95<norm<1.05 and 0<=m.pose.covariance[0]<.25 and -.2<time.time()-stamp<.7)
        state['valid']=valid
        if not valid:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if state['previous_yaw'] is not None:
            delta=angle_delta(yaw,state['previous_yaw'])
            if abs(delta)>.5:state['valid']=False;return
            state['total']+=delta
        state['position']=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]
        state['previous_yaw']=yaw;state['yaw']=yaw;state['pose_time']=time.monotonic()
        event(dict(kind='odom',stamp=stamp,yaw=yaw,total_yaw=state['total'],position=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z]))
    n.create_subscription(Odometry,'/stereo/odom',pose,qos_profile_sensor_data)
    pub=n.create_publisher(Twist,'/cmd_vel',1);enable=n.create_client(SetBool,'/jetbot/enable');stop=n.create_client(Trigger,'/jetbot/stop')
    def call(client,request):
        if not client.wait_for_service(timeout_sec=2):raise RuntimeError('Motor service unavailable')
        f=client.call_async(request);deadline=time.monotonic()+2
        while not f.done() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
        if not f.done() or not f.result().success:raise RuntimeError('Motor service rejected or timed out')
    def health(require_enabled=False):
        now=time.monotonic();s=state['status']
        if n.count_publishers('/cmd_vel')>1:raise RuntimeError('Another velocity controller is active')
        if any(now-t>.7 for t in frame_times.values()):raise RuntimeError('Camera stream stale')
        if state['origin'] is not None and math.dist(state['origin'],state['position'])>.2:raise RuntimeError('Rotation center drift exceeds 20cm')
        if not s or now-state['status_time']>.7:raise RuntimeError('Motor status stale')
        if s.get('fault') or s.get('dry_run'):raise RuntimeError('Motor fault or simulation: '+str(s))
        if require_enabled and not s.get('enabled'):raise RuntimeError('Motors disabled externally')
        if not state['valid'] or now-state['pose_time']>.5:raise TrackingUnavailable('Visual odometry lost/stale')
    started=time.time();result='aborted';reason='';armed=False
    def interrupt(*_):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupt)
    try:
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.03)
            if min(counts.values())>=10 and state['status'] and state['yaw'] is not None:break
        health()
        if state['status'].get('enabled'):raise RuntimeError('Another controller may be active; motors already enabled')
        state['total']=0.;state['origin']=state['position'][:]
        call(enable,SetBool.Request(data=True));armed=True
        # Status publication must acknowledge enabling before any nonzero command.
        deadline=time.monotonic()+1
        while not state['status'].get('enabled') and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
        health(True)
        goal=math.radians(a.degrees);deadline=time.monotonic()+a.timeout
        phase_end=0.;moving=False;last_progress=0.;recoveries=0
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.02)
            try:
                health(True)
            except TrackingUnavailable:
                pub.publish(Twist());moving=False;recoveries+=1
                if recoveries>72:raise RuntimeError('Too many tracking interruptions')
                recovery_end=time.monotonic()+3.;stable=0;last_pose=0.
                while time.monotonic()<recovery_end:
                    pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.02)
                    try:health(True)
                    except TrackingUnavailable:stable=0;continue
                    if state['pose_time']!=last_pose:stable+=1;last_pose=state['pose_time']
                    if stable>=4:break
                else:raise RuntimeError('Tracking did not recover while stopped')
                event(dict(kind='tracking_recovered',count=recoveries))
                phase_end=time.monotonic()+.9
                continue
            now=time.monotonic();progress=state['total']
            if progress<-.15:raise RuntimeError('Rotation direction differs from commanded direction')
            if progress>=goal-min(.08,goal*.02):
                result='completed_visual_yaw';break
            if now>=phase_end:
                moving=not moving
                phase_end=now+(.06 if moving else .9)
            m=Twist()
            if moving:m.angular.z=.25
            pub.publish(m)
            if progress-last_progress>math.radians(15):
                print('Visual yaw %.1f degrees'%math.degrees(progress),flush=True);last_progress=progress
        else:raise RuntimeError('Rotation time limit reached')
    except (Exception,KeyboardInterrupt) as e:
        reason=str(e) or 'Interrupted';print('Stopped:',reason,flush=True)
    finally:
        pub.publish(Twist())
        try:call(stop,Trigger.Request())
        except Exception as e:reason+='; stop service: '+str(e)
        # Record a stationary tail and the final measured heading after braking.
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
        report=dict(result=result,reason=reason,requested_degrees=a.degrees,visual_yaw_degrees=math.degrees(state['total']),frames=counts,start=started,end=time.time(),hardware_angle_feedback=False,note='Visual yaw estimate, not verified wheel angle or exact 360-degree coverage. Rotation may drift in translation.')
        (a.output/'survey.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2),flush=True)
        for f in streams.values():f.close()
        events.close();n.destroy_node();rclpy.shutdown()
    return 0 if result=='completed_visual_yaw' else 2

if __name__=='__main__':raise SystemExit(main())
