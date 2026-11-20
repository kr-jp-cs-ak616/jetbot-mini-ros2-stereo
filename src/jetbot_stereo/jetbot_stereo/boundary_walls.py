"""White-partition boundary proposals and floor-constrained wall geometry.

Metric geometry assumes measured camera height, pitch, roll and a flat floor.
Background pixels never become map points. This is an experimental wall model.
"""
import math
import cv2
import numpy as np


def preprocess(image):
    if image is None or image.ndim!=3:raise ValueError('BGR image required')
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    smooth=cv2.bilateralFilter(cv2.medianBlur(gray,3),7,30,5)
    contrast=cv2.createCLAHE(clipLimit=2.,tileGridSize=(8,8)).apply(smooth)
    edges=cv2.Canny(contrast,60,140)
    return dict(gray=smooth,contrast=contrast,edges=edges)


def boundary_pairs(image, *, allow_high_edges=False, allow_low_contrast=False,
                   min_panel_luma=75., candidate_filter=None):
    proc=preprocess(image);h,w=image.shape[:2]
    detected=cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD).detect(proc['contrast'])[0]
    if detected is None:return [],proc
    lines=[]
    min_edge_y=5 if allow_high_edges else h*.18
    for x1,y1,x2,y2 in detected[:,0]:
        if x2<x1:x1,x2,y1,y2=x2,x1,y2,y1
        if x2-x1<20 or abs((y2-y1)/(x2-x1))>.65 or min(y1,y2)<min_edge_y:continue
        lines.append(np.array([x1,y1,x2,y2],float))
    # Join nearly collinear pieces; do not extrapolate beyond observed endpoints.
    merged=[]
    for seed in sorted(lines,key=lambda x:-(x[2]-x[0])):
        m=(seed[3]-seed[1])/(seed[2]-seed[0]);b=seed[1]-m*seed[0]
        chosen=[l for l in lines if abs((l[3]-l[1])/(l[2]-l[0])-m)<.09 and
                max(abs(l[1]-m*l[0]-b),abs(l[3]-m*l[2]-b))<8]
        points=np.array([[l[0],l[1]] for l in chosen]+[[l[2],l[3]] for l in chosen])
        if len(points)<2:continue
        m,b=np.polyfit(points[:,0],points[:,1],1);lo=float(points[:,0].min());hi=float(points[:,0].max())
        if hi-lo<w*.12:continue
        if any(abs(m-t['m'])<.04 and abs(b-t['b'])<5 for t in merged):continue
        merged.append(dict(m=float(m),b=float(b),lo=lo,hi=hi))
    candidates=[];saturation=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)[:,:,1]
    texture = (np.abs(cv2.Laplacian(proc['gray'], cv2.CV_32F))
               if allow_low_contrast else None)
    for top in merged:
        for bottom in merged:
            lo=max(top['lo'],bottom['lo']);hi=min(top['hi'],bottom['hi'])
            if hi-lo<w*.12:continue
            xs=np.linspace(lo+2,hi-2,35);ty=top['m']*xs+top['b'];by=bottom['m']*xs+bottom['b']
            if min(by-ty)<h*.07 or max(by-ty)>h*.75 or min(ty)<5 or max(by)>h-20:continue
            x=np.rint(xs).astype(int)
            inside=np.concatenate([proc['gray'][np.rint(ty+(by-ty)*f).astype(int),x] for f in [.25,.5,.75]])
            sat=np.concatenate([saturation[np.rint(ty+(by-ty)*f).astype(int),x] for f in [.25,.5,.75]])
            near_bottom=proc['gray'][np.rint(by-8).astype(int),x].astype(float)
            floor=proc['gray'][np.rint(by+12).astype(int),x].astype(float)
            # The experiment uses light, low-saturation partition panels above a darker floor.
            contrast=float(np.median(near_bottom-floor))
            interior_variation=float(np.std(inside))
            if np.median(inside)<min_panel_luma or np.median(sat)>55 or interior_variation>27:continue
            if contrast<6:
                # On a smooth light panel the floor can have nearly the same brightness.
                # Recover the foot only when the floor is observably more textured.
                if (texture is None or contrast < -3 or interior_variation>12
                        or hi-lo<w*.35 or min(by-ty)<h*.28 or max(by)>h-25):
                    continue
                offsets=np.arange(5,25)
                below=texture[np.rint(by[:,None]+offsets).astype(int),x[:,None]]
                above=texture[np.rint(by[:,None]-offsets).astype(int),x[:,None]]
                texture_gain=np.mean(below-above,axis=1)
                if np.median(texture_gain)<.5 or (contrast<3 and np.mean(texture_gain>=.5)<.70):
                    continue
            score=(hi-lo)*(1+min(contrast,40)/40)/(1+interior_variation/30)
            candidate=dict(top=top,bottom=bottom,x0=int(math.ceil(lo)),x1=int(math.floor(hi)),score=float(score))
            if candidate_filter is None or candidate_filter(candidate):
                candidates.append(candidate)
    # At most one wall band per image column. Unknown columns stay unobserved.
    selected=[];occupied=np.zeros(w,bool)
    for c in sorted(candidates,key=lambda c:-c['score']):
        xs=np.arange(c['x0'],c['x1']+1)
        xs=xs[~occupied[xs]]
        if len(xs)<w*.1:continue
        # Separate non-contiguous parts so a hidden gap cannot become a wall.
        for part in np.split(xs,np.flatnonzero(np.diff(xs)>1)+1):
            if len(part)<w*.1:continue
            c=dict(c,x0=int(part[0]),x1=int(part[-1]));selected.append(c);occupied[part]=True
        if len(selected)>=4:break
    return sorted(selected,key=lambda c:c['x0']),proc


def wall_geometry(pair,projection,rectification,height_m,pitch_deg=0.):
    if not math.isfinite(height_m) or not math.isfinite(pitch_deg) or not .02<=height_m<=2 or abs(pitch_deg)>30:raise ValueError('Invalid camera mounting geometry')
    p=np.asarray(projection).reshape(3,4);r=np.asarray(rectification).reshape(3,3)
    pitch=math.radians(pitch_deg);c,s=math.cos(pitch),math.sin(pitch)
    camera_to_level=np.array([[1,0,0],[0,c,s],[0,-s,c]])
    def rays(u,v):
        rect=np.column_stack([(u-p[0,2])/p[0,0],(v-p[1,2])/p[1,1],np.ones_like(u)])
        return (rect@r)@camera_to_level.T
    u=np.linspace(pair['x0'],pair['x1'],32);bottom=pair['bottom']['m']*u+pair['bottom']['b']
    rays_bottom=rays(u,bottom)
    if np.any(rays_bottom[:,1]<=.015):raise ValueError('Floor boundary too close to/above horizon')
    foot=rays_bottom*(height_m/rays_bottom[:,1,None])
    if np.any(foot[:,2]<.15) or np.any(foot[:,2]>5):raise ValueError('Boundary range outside experimental limits')
    center=foot[:,[0,2]].mean(axis=0);_,_,vh=np.linalg.svd(foot[:,[0,2]]-center,full_matrices=False)
    normal2=vh[-1];offset=-normal2@center
    if offset<0:normal2=-normal2;offset=-offset
    n=np.array([normal2[0],0,normal2[1]])
    rt=rays(u,pair['top']['m']*u+pair['top']['b']);den=rt@n
    if np.any(den>=-.01):raise ValueError('Top ray does not hit the wall in front')
    tops=rt*(-offset/den[:,None]);heights=height_m-tops[:,1]
    if min(heights)<.015 or max(heights)>2:raise ValueError('Implausible panel height')
    # Geometry is returned in the original optical frame, consistent with odometry TF.
    foot_original=foot@camera_to_level
    return dict(normal_level=n.tolist(),offset_m=float(offset),floor_endpoints_optical=[foot_original[0].tolist(),foot_original[-1].tolist()],floor_endpoints_level=[foot[0].tolist(),foot[-1].tolist()],height_m=float(np.median(heights)),height_spread_m=float(np.ptp(heights)),distance_m=float(offset),camera_height_m=height_m,pitch_deg=pitch_deg,metric_source='flat_floor_height_and_boundary',metric_verified=False),rays


def model_partitions(image,projection,rectification,height_m,pitch_deg=0.,*,partition_height_m=None,
                     partition_pitch_tolerance_deg=0., min_panel_luma=75.):
    if partition_height_m is not None and (not math.isfinite(partition_height_m) or not .015<=partition_height_m<=2):
        raise ValueError('Invalid measured partition height')
    if not math.isfinite(partition_pitch_tolerance_deg) or not 0<=partition_pitch_tolerance_deg<=4:
        raise ValueError('Invalid partition pitch tolerance')
    if not 0<=min_panel_luma<=255:
        raise ValueError('Invalid minimum panel brightness')
    if image is None or image.ndim!=3:
        raise ValueError('BGR image required')
    h,w=image.shape[:2]

    def geometry_at(pair, pitch):
        return wall_geometry(pair,projection,rectification,height_m,pitch)

    def fit_near_partition(pair, nominal_height):
        # Retain the configured camera angle as a prior. Only fit pitch from a broad,
        # nearby panel whose height is known, since a narrow band is ambiguous.
        if nominal_height>partition_height_m:
            low=pitch_deg
            high=min(30.,pitch_deg+partition_pitch_tolerance_deg)
            bracketed=geometry_at(pair,high)[0]['height_m']<=partition_height_m
        else:
            low=max(-30.,pitch_deg-partition_pitch_tolerance_deg)
            high=pitch_deg
            bracketed=geometry_at(pair,low)[0]['height_m']>=partition_height_m
        if not bracketed:
            raise ValueError('Height does not match within the pitch tolerance')
        for _ in range(7):
            mid=(low+high)/2
            if geometry_at(pair,mid)[0]['height_m']>partition_height_m:
                low=mid
            else:
                high=mid
        wall,rays=geometry_at(pair,(low+high)/2)
        if wall['distance_m']>1.25 or wall['height_spread_m']>.15*partition_height_m:
            raise ValueError('Height fit requires a nearby panel with consistent edges')
        wall['metric_source']='known_partition_height_and_floor_boundary'
        return wall,rays

    def checked_geometry(pair):
        wall,rays=geometry_at(pair,pitch_deg)
        if partition_height_m is not None:
            # The two edges should give a plausible, consistent height over the observed span.
            tolerance=max(.015,.15*partition_height_m)
            if abs(wall['height_m']-partition_height_m)<=tolerance and wall['height_spread_m']<=2*tolerance:
                return wall,rays
            x=np.array([pair['x0'],pair['x1']])
            top=pair['top']['m']*x+pair['top']['b']
            bottom=pair['bottom']['m']*x+pair['bottom']['b']
            if (partition_pitch_tolerance_deg>0 and x[1]-x[0]>=w*.35
                    and min(bottom-top)>=h*.28):
                return fit_near_partition(pair,wall['height_m'])
            raise ValueError('Wall edges disagree with measured partition height')
        return wall,rays

    def plausible(pair):
        try:
            checked_geometry(pair)
        except ValueError:
            return False
        return True

    pairs,proc=boundary_pairs(image,allow_high_edges=partition_height_m is not None,
                              allow_low_contrast=partition_height_m is not None and partition_pitch_tolerance_deg>0,
                              min_panel_luma=min_panel_luma,
                              candidate_filter=plausible if partition_height_m is not None else None)
    masked=np.zeros_like(image);depth=np.zeros((h,w),np.float32);walls=[];preview=image.copy();keep=np.zeros((h,w),bool)
    for pair in pairs:
        try:wall,rays=checked_geometry(pair)
        except ValueError:continue
        u=np.arange(pair['x0'],pair['x1']+1)
        top=np.ceil(pair['top']['m']*u+pair['top']['b']).astype(int)
        bottom=np.floor(pair['bottom']['m']*u+pair['bottom']['b']).astype(int)
        for x,t,b in zip(u,top,bottom):
            # RGB above the panel is removed. Below it is the visible near-side floor.
            masked[t:,x]=image[t:,x];keep[t:,x]=True
            vs=np.arange(t,b+1);rs=rays(np.full(len(vs),x),vs)
            denominator=rs@np.array(wall['normal_level']);ds=-wall['offset_m']/denominator
            # Depth image uses rectified optical Z: each rectified ray's Z is one.
            good=(ds>.15)&(ds<5);depth[vs[good],x]=ds[good]
        polygon=np.array([[u[0],top[0]],[u[-1],top[-1]],[u[-1],bottom[-1]],[u[0],bottom[0]]],np.int32)
        cv2.polylines(preview,[polygon],True,(0,255,255),2)
        wall.update(x0=int(u[0]),x1=int(u[-1]),top=pair['top'],bottom=pair['bottom'])
        walls.append(wall)
    # Walls only: no invented floor depth or free space, and no behind-wall geometry.
    preview[~keep]=0
    cv2.putText(preview,'Wall hypotheses | floor-height model; pitch not independently verified',(5,20),cv2.FONT_HERSHEY_SIMPLEX,.38,(0,255,255),1)
    return dict(walls=walls,masked_rgb=masked,depth=depth,keep_mask=keep,preview=preview,preprocessing=proc)


class SparseWallMap:
    """Store bounded wall cells only; unobserved and behind-wall space allocates no cells."""
    def __init__(self,resolution=.02,max_cells=50000):
        self.resolution=resolution;self.max_cells=max_cells;self.cells=set()
    def add_segment(self,a,b):
        a=np.asarray(a,float);b=np.asarray(b,float)
        count=max(2,int(np.linalg.norm(b-a)/self.resolution*2)+1)
        if count>10000:raise ValueError('Wall segment too large')
        cells={tuple(np.floor(v/self.resolution).astype(int)) for v in np.linspace(a,b,count)}
        if len(self.cells|cells)>self.max_cells:raise ValueError('Sparse wall map memory limit reached')
        self.cells.update(cells)
    def serialize(self):
        return dict(resolution=self.resolution,occupied_cells=sorted([[int(v) for v in c] for c in self.cells]),unknown_space='implicit; no cells allocated',free_space_inferred=False)


class ObservationBuffer:
    """Bounded timestamp association, independent of processing/arrival latency."""
    def __init__(self,capacity=20):
        from collections import deque
        self.items=deque(maxlen=capacity)
    def add(self,stamp,payload):
        self.items.append((stamp,payload))
    def match(self,stamp,tolerance=.025):
        if not self.items:return None
        index=min(range(len(self.items)),key=lambda i:abs(self.items[i][0]-stamp))
        if abs(self.items[index][0]-stamp)>tolerance:return None
        result=self.items[index];del self.items[index];return result
