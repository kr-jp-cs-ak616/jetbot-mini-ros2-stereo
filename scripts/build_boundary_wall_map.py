#!/usr/bin/env python3
"""Preview a compact partition-wall map from an existing rectified DB frame."""
import argparse,json,sqlite3
from pathlib import Path
import cv2,numpy as np,yaml
from jetbot_stereo.boundary_walls import model_partitions,SparseWallMap

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--database',type=Path,required=True);p.add_argument('--node-id',type=int,default=149)
p.add_argument('--output',type=Path,required=True);p.add_argument('--height-cm',type=float,required=True)
p.add_argument('--pitch-deg',type=float,default=0.);p.add_argument('--calibration-dir',type=Path,default=Path('calibration'))
a=p.parse_args()
if (a.output/'wall-map.json').exists():raise ValueError('Existing wall map is preserved; choose another output folder')
a.output.mkdir(parents=True,exist_ok=True);cv2.setNumThreads(2)
c=sqlite3.connect(a.database.resolve().as_uri()+'?mode=ro',uri=True)
row=c.execute('select image from Data where id=?',(a.node_id,)).fetchone();c.close()
if row is None:raise ValueError('Node not found')
im=cv2.imdecode(np.frombuffer(row[0],np.uint8),1)
if im is None or im.shape[:2]!=(480,640):raise ValueError('Expected rectified 640x480 stereo DB image')
d=yaml.safe_load((a.calibration_dir/'left.yaml').read_text())
m=model_partitions(im,d['projection_matrix']['data'],d['rectification_matrix']['data'],a.height_cm/100,a.pitch_deg)
for key in ('contrast','edges'):
    m['preprocessing'][key][~m['keep_mask']]=0
for name,image in [('preprocessed',m['preprocessing']['contrast']),('edges',m['preprocessing']['edges']),('boundaries',m['preview']),('map-input-masked',m['masked_rgb'])]:cv2.imwrite(str(a.output/(name+'.png')),image)
comparison=np.vstack([np.hstack([cv2.cvtColor(m['preprocessing']['contrast'],cv2.COLOR_GRAY2BGR),cv2.cvtColor(m['preprocessing']['edges'],cv2.COLOR_GRAY2BGR)]),np.hstack([m['preview'],m['masked_rgb']])]);cv2.imwrite(str(a.output/'processing-preview.jpg'),comparison)
wallmap=SparseWallMap()
for wall in m['walls']:
 endpoints=np.array(wall['floor_endpoints_level'])[:,[0,2]];wallmap.add_segment(endpoints[0],endpoints[1])
result=dict(camera_height_cm=a.height_cm,assumed_pitch_deg=a.pitch_deg,metric_verified=False,source_node=a.node_id,walls=m['walls'],sparse_map=wallmap.serialize(),background_pixels_stored=False,pose='local camera origin; not merged into old RTAB-Map DB')
(a.output/'wall-map.json').write_text(json.dumps(result,indent=2)+'\n')
canvas=np.full((650,800,3),235,np.uint8);origin=np.array([400,550]);scale=350
for wall in m['walls']:
 points=np.array(wall['floor_endpoints_level'])[:,[0,2]];xy=np.column_stack([origin[0]+points[:,0]*scale,origin[1]-points[:,1]*scale]).astype(int)
 cv2.line(canvas,tuple(xy[0]),tuple(xy[1]),(0,0,0),5)
cv2.circle(canvas,tuple(origin),7,(0,160,0),-1)
cv2.putText(canvas,'Estimated partition walls | h=%.1fcm, pitch=%.1fdeg'%(a.height_cm,a.pitch_deg),(15,30),cv2.FONT_HERSHEY_SIMPLEX,.6,(20,20,20),1)
cv2.putText(canvas,'Grey: unknown. Background geometry is not stored.',(15,60),cv2.FONT_HERSHEY_SIMPLEX,.55,(20,20,20),1)
cv2.imwrite(str(a.output/'wall-map-preview.png'),canvas)
print(json.dumps({'walls':len(m['walls']),'occupied_cells':len(wallmap.cells),'map_json_bytes':(a.output/'wall-map.json').stat().st_size,'output':str(a.output.resolve()),'metric_verified':False},indent=2))
