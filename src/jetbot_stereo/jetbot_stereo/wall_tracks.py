"""Associate repeated world-frame wall observations with persistent IDs."""
from collections import deque
import math
import numpy as np


def line_model(points):
    points=np.asarray(points,float)
    if points.shape!=(2,2) or not np.isfinite(points).all():raise ValueError('Finite XY endpoints required')
    direction=points[1]-points[0];length=float(np.linalg.norm(direction))
    if length<.05:raise ValueError('Wall too short')
    direction/=length
    if direction[0]<0 or (abs(direction[0])<1e-8 and direction[1]<0):direction=-direction
    normal=np.array([-direction[1],direction[0]])
    return direction,normal,float(-normal@points.mean(axis=0))


def match_score(a,b,angle_deg=7.,distance=.06,gap=.10):
    u,n,d=line_model(a);v,m,e=line_model(b)
    angle=math.acos(float(np.clip(abs(u@v),-1,1)))
    if angle>math.radians(angle_deg):return None
    separation=max(abs(n@np.mean(b,axis=0)+d),abs(m@np.mean(a,axis=0)+e))
    if separation>distance:return None
    ia=np.sort(np.asarray(a)@u);ib=np.sort(np.asarray(b)@u)
    interval_gap=max(0.,float(max(ia[0],ib[0])-min(ia[1],ib[1])))
    if interval_gap>gap:return None
    return separation/distance+angle/math.radians(angle_deg)+interval_gap/gap


class WallTrack:
    def __init__(self,identifier,observation,stamp):
        self.id=identifier;self.points=np.array(observation['endpoints_xy_m'],float)
        line_model(self.points)
        self.heights=deque([float(observation['height_m'])],maxlen=15)
        self.floor=float(observation.get('floor_z_m',0.));self.hits=1;self.last_stamp=stamp
    def update(self,observation,stamp):
        points=np.array(observation['endpoints_xy_m'],float)
        u,_,_=line_model(self.points);v,_,_=line_model(points)
        if u@v<0:v=-v
        direction=.8*u+.2*v;direction/=np.linalg.norm(direction)
        normal=np.array([-direction[1],direction[0]])
        # Smooth the wall plane but retain already observed portions along it.
        offset=.8*float(normal@self.points.mean(axis=0))+.2*float(normal@points.mean(axis=0))
        along=np.concatenate([self.points@direction,points@direction])
        self.points=np.array([along.min()*direction+offset*normal,along.max()*direction+offset*normal])
        self.floor=.8*self.floor+.2*float(observation.get('floor_z_m',self.floor))
        self.heights.append(float(observation['height_m']))
        if stamp>self.last_stamp+1e-6:self.hits+=1
        self.last_stamp=max(self.last_stamp,stamp)
    def serialize(self):
        return dict(id=self.id,endpoints_xy_m=self.points.tolist(),height_m=float(np.median(self.heights)),floor_z_m=self.floor,observations=self.hits,source='associated_floor_boundary_hypothesis')


class WallTracks:
    def __init__(self,max_tracks=256,min_observations=3):
        self.tracks={};self.next_id=0;self.max_tracks=max_tracks;self.minimum=min_observations
        self.last_stamp=-float('inf')
    def observe(self,observations,stamp):
        if not math.isfinite(stamp) or stamp<=self.last_stamp:return
        self.last_stamp=stamp
        # Unconfirmed candidates expire; confirmed walls survive leaving the view.
        for identifier,t in list(self.tracks.items()):
            if t.hits<self.minimum and stamp-t.last_stamp>2.:del self.tracks[identifier]
        for observation in observations:
            points=np.asarray(observation['endpoints_xy_m'],float)
            try:line_model(points)
            except ValueError:continue
            scores=[(score,identifier) for identifier,t in self.tracks.items()
                    if (score:=match_score(t.points,points)) is not None]
            if scores:self.tracks[min(scores)[1]].update(observation,stamp)
            elif len(self.tracks)<self.max_tracks:
                self.tracks[self.next_id]=WallTrack(self.next_id,observation,stamp);self.next_id+=1
        # A later spanning observation can join formerly separate fragments.
        changed=True
        while changed:
            changed=False
            ids=sorted(self.tracks)
            for i,first in enumerate(ids):
                for second in ids[i+1:]:
                    a,b=self.tracks[first],self.tracks[second]
                    if match_score(a.points,b.points,angle_deg=5.,distance=.04,gap=.05) is None:continue
                    hits=max(a.hits,b.hits);last=max(a.last_stamp,b.last_stamp)
                    a.update(b.serialize(),last);a.hits=hits;a.last_stamp=last
                    a.heights.extend(b.heights);del self.tracks[second];changed=True;break
                if changed:break
    def confirmed(self):
        return [t.serialize() for _,t in sorted(self.tracks.items()) if t.hits>=self.minimum]


class RelativeHeading:
    """Unwrap consecutive visual-odometry yaw; no motor-angle measurement implied."""
    def __init__(self):self.previous=None;self.total=0.
    def update(self,yaw):
        if not math.isfinite(yaw):raise ValueError('Finite yaw required')
        delta=0. if self.previous is None else math.atan2(math.sin(yaw-self.previous),math.cos(yaw-self.previous))
        self.previous=yaw;self.total+=delta
        return dict(relative_yaw_deg=math.degrees(self.total),frame_rotation_deg=math.degrees(delta),rotation_direction='left' if delta>math.radians(.3) else 'right' if delta<math.radians(-.3) else 'stationary')


class PoseContinuity:
    """Do not create new walls from discontinuous/reinitialized visual poses."""
    def __init__(self):self.previous=None
    def accept(self,stamp,position,yaw):
        position=np.asarray(position,float)
        if position.shape!=(3,) or not np.isfinite(position).all() or not math.isfinite(yaw):return False
        if self.previous is not None:
            old_stamp,old_position,old_yaw=self.previous;dt=stamp-old_stamp
            if dt<=0:return False
            turn=abs(math.atan2(math.sin(yaw-old_yaw),math.cos(yaw-old_yaw)))
            if np.linalg.norm(position-old_position)>min(.3,.08+.3*dt) or turn>min(.8,.15+2.5*dt):return False
        self.previous=(stamp,position.copy(),yaw);return True
