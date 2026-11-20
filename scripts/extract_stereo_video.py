#!/usr/bin/env python3
"""Select distinct, stationary stereo frames using recorded shared-clock times."""
import argparse
from bisect import bisect_left
import csv
import json
from pathlib import Path
import cv2
import numpy as np
from calibrate_stereo import corners


def read_video(path):
    capture=cv2.VideoCapture(str(path))
    frames=[]
    while True:
        ok,frame=capture.read()
        if not ok:
            break
        frames.append(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY))
    capture.release()
    return frames


def motion(frames,index,points):
    if index<1 or index>=len(frames)-1:
        return float('inf')
    a,sa,_=cv2.calcOpticalFlowPyrLK(frames[index],frames[index-1],points,None)
    b,sb,_=cv2.calcOpticalFlowPyrLK(frames[index],frames[index+1],points,None)
    if a is None or b is None:
        return float('inf')
    good=(sa.ravel()>0)&(sb.ravel()>0)
    if good.sum()<44:
        return float('inf')
    return float(max(np.median(np.linalg.norm(a[good]-points[good],axis=2)),
                     np.median(np.linalg.norm(b[good]-points[good],axis=2))))


def extract(root,square_mm=25,stride=5,temporal_error=None,output_name='frame-pairs'):
    videos=[read_video(root/f'camera-{sensor}.avi') for sensor in (0,1)]
    tables=[]
    for sensor in (0,1):
        with (root/f'camera-{sensor}-timestamps.csv').open() as stream:
            tables.append([dict(index=int(row['frame_index']),time=int(row['clock_time_ns']))
                           for row in csv.DictReader(stream)
                           if int(row['clock_time_ns'])>0])
    output=root/output_name
    if output.exists():
        raise ValueError('프레임 추출 폴더가 이미 있습니다: '+str(output))
    output.mkdir()
    (output/'capture-config.json').write_text(json.dumps(dict(square_mm=square_mm,inner_corners=[8,6]),indent=2)+'\n')
    right_times=[r['time'] for r in tables[1]]
    selected=[]
    counts=dict(no_time_match=0,pattern_failed=0,moving=0,blurred=0,duplicate=0)
    for left in tables[0][::stride]:
        i=left['index']
        if i>=len(videos[0]) or not right_times:
            continue
        pos=bisect_left(right_times,left['time'])
        candidates=[v for v in (pos-1,pos) if 0<=v<len(right_times)]
        match=min(candidates,key=lambda v:abs(right_times[v]-left['time']))
        right=tables[1][match]
        j=right['index']
        skew=abs(right['time']-left['time'])/1e6
        if skew>35 or j>=len(videos[1]):
            counts['no_time_match']+=1;continue
        imgs=[videos[0][i],videos[1][j]]
        pts=[corners(img) for img in imgs]
        if any(p is None for p in pts):
            counts['pattern_failed']+=1;continue
        movement=[motion(frames,index,p) for frames,index,p in zip(videos,(i,j),pts)]
        predicted_error=max(movement)*skew/(1000/15)
        if (not np.isfinite(max(movement)) or
                (temporal_error is None and max(movement)>1) or
                (temporal_error is not None and predicted_error>temporal_error)):
            counts['moving']+=1;continue
        sharp=[]
        for img,p in zip(imgs,pts):
            mask=np.zeros_like(img)
            cv2.fillConvexPoly(mask,cv2.convexHull(p).astype('int32'),255)
            sharp.append(float(cv2.Laplacian(img,cv2.CV_64F)[mask>0].var()))
        if min(sharp)<250:
            counts['blurred']+=1;continue
        if any(max(float(np.sqrt(np.mean((a-b)**2))) for a,b in zip(pts,old['points']))<5
               for old in selected):
            counts['duplicate']+=1;continue
        folder=output/f'frame-{i:05d}'
        folder.mkdir()
        for sensor,img in enumerate(imgs):
            cv2.imwrite(str(folder/f'camera-{sensor}.jpg'),img)
        meta=dict(left_index=i,right_index=j,skew_ms=skew,motion_px_per_frame=movement,
                  predicted_temporal_error_px=predicted_error,
                  sharpness=sharp,clock_time_ns=[left['time'],right['time']])
        (folder/'source.json').write_text(json.dumps(meta,indent=2)+'\n')
        (folder/'status.txt').write_text('0\n')
        selected.append(dict(points=pts,folder=str(folder),metadata=meta))
    report=dict(video_frames=[len(v) for v in videos],selected_pairs=len(selected),
                excluded=counts,max_skew_ms=35,max_motion_px_per_frame=1 if temporal_error is None else None,
                max_predicted_temporal_error_px=temporal_error,stride=stride,
                note='Heuristic frame selection; timestamps are buffer presentation times, not hardware exposure synchronization.',
                pairs=[dict(folder=r['folder'],**r['metadata']) for r in selected])
    report_name='selection-report.json' if output_name=='frame-pairs' else f'selection-report-{output_name}.json'
    (root/report_name).write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='pairs'},ensure_ascii=False,indent=2))
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video_folder',type=Path)
    parser.add_argument('--square-mm',type=float,default=25)
    parser.add_argument('--stride',type=int,default=5)
    parser.add_argument('--max-temporal-error-px',type=float,default=None)
    parser.add_argument('--output-name',default='frame-pairs')
    args=parser.parse_args()
    if args.stride<1 or not np.isfinite(args.square_mm) or args.square_mm<=0:
        parser.error('칸 크기와 stride는 양수여야 합니다')
    if args.max_temporal_error_px is not None and (not np.isfinite(args.max_temporal_error_px) or args.max_temporal_error_px<=0):
        parser.error('시간차에 따른 오차 한계는 양수여야 합니다')
    if Path(args.output_name).name!=args.output_name or args.output_name in ('','.','..'):
        parser.error('output-name은 폴더 이름 하나만 지정하세요')
    extract(args.video_folder,args.square_mm,args.stride,args.max_temporal_error_px,args.output_name)
