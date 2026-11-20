#!/usr/bin/env python3
"""Package recorded rotation segments and offline wall-analysis results."""
import argparse,json,shutil
from pathlib import Path
import cv2,numpy as np
p=argparse.ArgumentParser(description=__doc__);p.add_argument('segments',type=Path,nargs='+');a=p.parse_args()
summary={'completed_360':False,'segments':[],'total_visual_yaw_degrees':0,'semantic_wall_detection_validated':False,'missing_depth_is_free_space':False}
for root in a.segments:
 events=[json.loads(l) for l in (root/'events.jsonl').read_text().splitlines()]
 rows={s:[e for e in events if e.get('side')==s] for s in ('left','right')}
 for s,rs in rows.items():
  fps=(len(rs)-1)/(rs[-1]['stamp']-rs[0]['stamp'])
  v=cv2.VideoWriter(str(root/(s+'.avi')),cv2.VideoWriter_fourcc(*'MJPG'),fps,(640,480))
  with (root/(s+'.mjpeg')).open('rb') as f:
   for e in rs:
    f.seek(e['offset']);im=cv2.imdecode(np.frombuffer(f.read(e['length']),np.uint8),1);v.write(im)
  v.release()
 cal=root/'calibration-used';cal.mkdir(exist_ok=True)
 for name in ('left.yaml','right.yaml','mounting-status.json'):shutil.copy2(Path('calibration')/name,cal/name)
 survey=json.loads((root/'survey.json').read_text());report=json.loads((root/'wall-analysis/report.json').read_text())
 values=[r['valid_depth_fraction'] for r in report['frames'] if 'valid_depth_fraction' in r]
 record=dict(folder=str(root.resolve()),survey=survey,analyzed_pairs=len(values),depth_fraction_median=float(np.median(values)) if values else None,depth_fraction_max=max(values,default=0),vertical_plane_candidates=sum(len(r.get('planes',[])) for r in report['frames']))
 summary['segments'].append(record);summary['total_visual_yaw_degrees']+=survey['visual_yaw_degrees']
Path('log/rotation-wall-survey-summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
