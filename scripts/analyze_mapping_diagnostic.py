import json,cv2,numpy as np,yaml
from pathlib import Path
import argparse
parser=argparse.ArgumentParser(description='Offline timing and exploratory stereo geometry checks')
parser.add_argument('recording',type=Path)
args=parser.parse_args()
p=args.recording;events=[json.loads(l) for l in (p/'events.jsonl').read_text().splitlines()]
rows={s:[e for e in events if e.get('side')==s] for s in ['left','right']}
maps={}
for s in rows:
 d=yaml.safe_load(Path('calibration/'+s+'.yaml').read_text())
 maps[s]=cv2.initUndistortRectifyMap(np.array(d['camera_matrix']['data']).reshape(3,3),np.array(d['distortion_coefficients']['data']),np.array(d['rectification_matrix']['data']).reshape(3,3),np.array(d['projection_matrix']['data']).reshape(3,4),(640,480),cv2.CV_32FC1)
def read(s,e):
 with (p/(s+'.mjpeg')).open('rb') as f:f.seek(e['offset']);b=f.read(e['length'])
 return cv2.imdecode(np.frombuffer(b,np.uint8),1)
deltas=[min(abs(l['stamp']-r['stamp']) for r in rows['right'])*1000 for l in rows['left']]
stats=[];montage=[]
for idx in [0,150,300]:
 l=rows['left'][idx];r=min(rows['right'],key=lambda r:abs(l['stamp']-r['stamp']))
 a=cv2.remap(read('left',l),*maps['left'],cv2.INTER_LINEAR);b=cv2.remap(read('right',r),*maps['right'],cv2.INTER_LINEAR)
 sift=cv2.SIFT_create(nfeatures=2000);k1,d1=sift.detectAndCompute(a,None);k2,d2=sift.detectAndCompute(b,None)
 matches=[m for m,n in cv2.BFMatcher().knnMatch(d1,d2,k=2) if m.distance<.65*n.distance]
 x=np.float32([k1[m.queryIdx].pt for m in matches]);y=np.float32([k2[m.trainIdx].pt for m in matches])
 F,mask=cv2.findFundamentalMat(x,y,cv2.FM_RANSAC,1.0,.999)
 keep=mask.ravel().astype(bool);dy=np.abs(x[keep,1]-y[keep,1]);dx=x[keep,0]-y[keep,0]
 stats.append(dict(seconds=round(l['stamp']-rows['left'][0]['stamp'],2),matches=len(matches),ransac_inliers=int(sum(keep)),median_vertical_error_px=float(np.median(dy)),p90_vertical_error_px=float(np.percentile(dy,90)),median_disparity_px=float(np.median(dx))))
 combo=np.hstack([a,b])
 for yy in range(40,480,60):cv2.line(combo,(0,yy),(1279,yy),(0,255,0),1)
 montage.append(combo)
 cv2.imwrite(str(p/('rectified-%d.jpg'%idx)),combo)
result=dict(recording=json.loads((p/'recording.json').read_text()),timing_ms=dict(min=float(min(deltas)),median=float(np.median(deltas)),p95=float(np.percentile(deltas,95)),within_20ms_percent=float(np.mean(np.array(deltas)<=20)*100)),geometry=stats,geometry_warning='Sparse matches and repeated textures can produce false correspondences. Geometry numbers are exploratory, not calibration validation.')
(p/'analysis.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
# Convert the saved JPEG stream after recording; no extra CSI camera access.
for s,rs in rows.items():
 fps=(len(rs)-1)/(rs[-1]['stamp']-rs[0]['stamp']);v=cv2.VideoWriter(str(p/(s+'.avi')),cv2.VideoWriter_fourcc(*'MJPG'),fps,(640,480))
 for e in rs:v.write(read(s,e))
 v.release()
