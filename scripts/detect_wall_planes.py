#!/usr/bin/env python3
"""Offline stereo depth and vertical-plane candidates; missing depth stays unknown."""
import argparse,json,math
from pathlib import Path
import cv2,numpy as np,yaml


def fit_vertical_planes(points,seed=4,minimum=120):
    """Fit in original left optical frame; assumes camera mounted approximately level."""
    points=np.asarray(points,dtype=float)
    points=points[np.isfinite(points).all(axis=1)]
    rng=np.random.default_rng(seed)
    if len(points)>5000:points=points[rng.choice(len(points),5000,replace=False)]
    planes=[]
    for _ in range(3):
        if len(points)<minimum:break
        best=None
        for trial in range(250):
            a,b,c=points[rng.choice(len(points),3,replace=False)]
            normal=np.cross(b-a,c-a);norm=np.linalg.norm(normal)
            if norm<1e-8:continue
            normal/=norm
            if abs(normal[1])>math.sin(math.radians(20)):continue
            offset=-normal@a;residual=np.abs(points@normal+offset)
            inliers=residual<.035;score=int(inliers.sum())
            if score>=minimum and (best is None or score>best[0]):best=(score,inliers)
        if best is None:break
        selected=points[best[1]];center=selected.mean(axis=0)
        _,_,vh=np.linalg.svd(selected-center,full_matrices=False);normal=vh[-1];offset=-normal@center
        residual=np.abs(points@normal+offset);inliers=residual<.035;selected=points[inliers]
        if normal[2]>0:normal=-normal;offset=-offset
        tangent=np.cross([0,1,0],normal);tangent/=np.linalg.norm(tangent)
        width=float(np.ptp(np.percentile(selected@tangent,[5,95])))
        height=float(np.ptp(np.percentile(selected[:,1],[5,95])))
        if abs(normal[1])<=math.sin(math.radians(20)) and width>=.5 and height>=.35:
            planes.append(dict(type='vertical_plane_candidate',normal=normal.tolist(),offset_m=float(offset),perpendicular_distance_m=float(abs(offset)),support_points=int(inliers.sum()),width_m=width,height_m=height,median_residual_m=float(np.median(residual[inliers])),semantic_wall_confirmed=False))
        points=points[~inliers]
    return planes


def analyze_pair(left,right,calibration):
    maps=[];data=[]
    for s in ('left','right'):
        d=yaml.safe_load((calibration/(s+'.yaml')).read_text());data.append(d)
        maps.append(cv2.initUndistortRectifyMap(np.array(d['camera_matrix']['data']).reshape(3,3),np.array(d['distortion_coefficients']['data']),np.array(d['rectification_matrix']['data']).reshape(3,3),np.array(d['projection_matrix']['data']).reshape(3,4),(640,480),cv2.CV_32FC1))
    grays=[cv2.cvtColor(cv2.remap(im,*m,cv2.INTER_LINEAR),cv2.COLOR_BGR2GRAY) for im,m in zip((left,right),maps)]
    options=dict(numDisparities=256,blockSize=5,P1=8*25,P2=32*25,disp12MaxDiff=1,uniquenessRatio=15,speckleWindowSize=100,speckleRange=2,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    padded=[cv2.copyMakeBorder(im,0,0,256,256,cv2.BORDER_CONSTANT,value=0) for im in grays]
    # Keep the full image search ROI; physical correspondence bounds below
    # exclude matches into padding, and LR consistency rejects unsupported matches.
    dl=cv2.StereoSGBM_create(minDisparity=0,**options).compute(*padded)[:,256:896].astype(np.float32)/16
    dr=cv2.StereoSGBM_create(minDisparity=-255,**options).compute(padded[1],padded[0])[:,256:896].astype(np.float32)/16
    yy,xx=np.indices(dl.shape);xr=np.rint(xx-dl).astype(int);inside=(xr>=0)&(xr<640)
    lr=np.abs(dl+dr[yy,np.clip(xr,0,639)])<1.5
    texture=cv2.boxFilter(grays[0].astype(np.float32)**2,-1,(9,9))-cv2.boxFilter(grays[0].astype(np.float32),-1,(9,9))**2
    valid=(dl>1)&(dl<255)&inside&lr&(texture>36)
    p0=np.array(data[0]['projection_matrix']['data']).reshape(3,4);p1=np.array(data[1]['projection_matrix']['data']).reshape(3,4)
    baseline=p0[0,3]/p0[0,0]-p1[0,3]/p1[0,0]
    depth=p0[0,0]*baseline/np.maximum(dl,.001);valid&=(depth>=.4)&(depth<=5)
    xyz=np.stack([(xx-p0[0,2])*depth/p0[0,0],(yy-p0[1,2])*depth/p0[1,1],depth],axis=-1)
    rotation=np.array(data[0]['rectification_matrix']['data']).reshape(3,3)
    points=(xyz[valid]@rotation)[::3]
    planes=fit_vertical_planes(points) if valid.mean()>=.03 else []
    view=cv2.cvtColor(grays[0],cv2.COLOR_GRAY2BGR)
    depth_color=cv2.applyColorMap(np.uint8(np.clip(depth/5*255,0,255)),cv2.COLORMAP_TURBO);depth_color[~valid]=0
    for i,plane in enumerate(planes):
        original=xyz@rotation;mask=valid&(np.abs(original@np.array(plane['normal'])+plane['offset_m'])<.035)
        view[mask]=(.4*view[mask]+.6*np.array([0,180,255])).astype(np.uint8)
        cv2.putText(view,'plane %d: %.2fm perpendicular'%(i+1,plane['perpendicular_distance_m']),(10,25+25*i),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,255,255),1)
    cv2.putText(depth_color,'Black: unknown depth',(10,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
    return dict(valid_depth_fraction=float(valid.mean()),planes=planes,unknown_is_free_space=False,insufficient_depth=bool(valid.mean()<.03),assumption='Camera approximately level; planes may be walls, cabinets, screens or furniture.'),np.hstack([view,depth_color])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('recording',type=Path);p.add_argument('--calibration-dir',type=Path,default=Path('calibration'));p.add_argument('--interval',type=float,default=1.)
    a=p.parse_args();cv2.setNumThreads(2)
    if not math.isfinite(a.interval) or a.interval<=0:p.error('interval must be positive')
    out=a.recording/'wall-analysis';out.mkdir(exist_ok=False)
    events=[json.loads(l) for l in (a.recording/'events.jsonl').read_text().splitlines()]
    frames={s:[e for e in events if e.get('side')==s] for s in ['left','right']}
    if not frames['left'] or not frames['right']:raise ValueError('Both camera streams are required')
    def read(s,e):
        with (a.recording/(s+'.mjpeg')).open('rb') as f:f.seek(e['offset']);b=f.read(e['length'])
        return cv2.imdecode(np.frombuffer(b,np.uint8),1)
    poses=[e for e in events if e.get('kind')=='odom' and 'total_yaw' in e]
    results=[];last=-float('inf');writer=cv2.VideoWriter(str(out/'wall-candidates.avi'),cv2.VideoWriter_fourcc(*'MJPG'),1/a.interval,(1280,480))
    try:
        for l in frames['left']:
            if l['stamp']-last<a.interval:continue
            if poses:
                nearby=[v['total_yaw'] for v in poses if abs(v['stamp']-l['stamp'])<=.15]
                if len(nearby)<3 or max(nearby)-min(nearby)>.025:continue
            r=min(frames['right'],key=lambda r:abs(l['stamp']-r['stamp']));skew=abs(l['stamp']-r['stamp'])
            if skew>.02:
                results.append(dict(stamp=l['stamp'],status='unknown_time_skew',skew_ms=skew*1000));continue
            last=l['stamp'];result,preview=analyze_pair(read('left',l),read('right',r),a.calibration_dir)
            result.update(stamp=l['stamp'],skew_ms=skew*1000,status='analyzed');results.append(result);writer.write(preview)
            cv2.imwrite(str(out/('frame-%05d.jpg'%l['index'])),preview)
    finally:writer.release()
    report=dict(frames=results,plane_detection_is_semantic_wall_detection=False,coverage_requires_valid_pose=True)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Analyzed',sum(r['status']=='analyzed' for r in results),'pairs; candidates',sum(len(r.get('planes',[])) for r in results),flush=True)

if __name__=='__main__':main()
