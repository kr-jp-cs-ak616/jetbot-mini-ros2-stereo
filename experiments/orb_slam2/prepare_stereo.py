#!/usr/bin/env python3
"""Prepare timestamp-paired, rectified images and metric settings for ORB-SLAM2."""
import argparse
from bisect import bisect_left
from contextlib import ExitStack
import json
from pathlib import Path

import cv2
import numpy as np
import yaml


def camera_models(directory):
    result=[]
    for side in ('left', 'right'):
        doc = yaml.safe_load((directory / f'{side}.yaml').read_text())
        if (doc['image_width'], doc['image_height']) != (640,480):
            raise ValueError('Expected 640x480 input calibration')
        result.append(tuple(np.asarray(doc[key]['data'], dtype=float).reshape(shape)
                            for key, shape in (('camera_matrix',(3,3)), ('distortion_coefficients',(-1,)),
                                               ('rectification_matrix',(3,3)), ('projection_matrix',(3,4)))))
    return result


def synchronized(directory, step_ns, tolerance_ns):
    samples = {'left':[], 'right':[]}
    with (directory/'events.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['kind'] != 'frame':
                continue
            if 'capture_ns' in row:
                stamp = int(row['capture_ns'])
            elif 'stamp' in row:
                stamp = int(round(row['stamp'] * 1e9))
            else:
                raise ValueError('Recording contains no camera timestamps')
            if stamp > 0:
                samples[row['side']].append((stamp, row))
    left,right=(sorted(rows, key=lambda item:item[0]) for rows in samples.values())
    right_times=[item[0] for item in right]
    used=set()
    next_time=0
    for stamp,row in left:
        if stamp < next_time:
            continue
        idx = bisect_left(right_times, stamp)
        options=[n for n in (idx-1,idx) if 0 <= n < len(right) and n not in used]
        if not options:
            continue
        nearest=min(options,key=lambda j:abs(right_times[j]-stamp))
        rs,right_row=right[nearest]
        if abs(rs-stamp)>tolerance_ns:
            continue
        used.add(nearest)
        next_time=stamp+step_ns
        yield (stamp+rs)//2,abs(stamp-rs),row,right_row


def write_settings(output, models, fps):
    left,right=(m[3] for m in models)
    fx,fy,cx,cy=left[0,0],left[1,1],left[0,2],left[1,2]
    if not np.allclose(left[:,:3],right[:,:3],atol=1e-4) or abs(right[1,3]-left[1,3]) > 1e-5:
        raise ValueError('Expected horizontal rectification with aligned principal points')
    bf=float(left[0,3]-right[0,3])
    if bf <= 0:
        raise ValueError('Left/right baseline sign is invalid')
    values={'Camera.fx':fx,'Camera.fy':fy,'Camera.cx':cx,'Camera.cy':cy,'Camera.bf':bf,
            'Camera.k1':0.0,'Camera.k2':0.0,'Camera.p1':0.0,'Camera.p2':0.0,
            'Camera.width':640,'Camera.height':480,'Camera.fps':float(fps),'Camera.RGB':0,
            'ThDepth':40.0,'ORBextractor.nFeatures':900,'ORBextractor.scaleFactor':1.2,
            'ORBextractor.nLevels':8,'ORBextractor.iniThFAST':15,'ORBextractor.minThFAST':5}
    (output/'settings.yaml').write_text('%YAML:1.0\n---\n' + ''.join(f'{k}: {v}\n' for k,v in values.items()))
    return bf/fx


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('calibration',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--min-frame-gap-ms',type=float,default=99.)
    parser.add_argument('--max-skew-ms',type=float,default=20.)
    args=parser.parse_args()
    if args.min_frame_gap_ms<=0 or not 0<args.max_skew_ms<=35:
        parser.error('Invalid frame interval or skew')
    cv2.setNumThreads(2)
    models=camera_models(args.calibration)
    maps=[cv2.initUndistortRectifyMap(*m,(640,480),cv2.CV_16SC2) for m in models]
    pairs=list(synchronized(args.recording,args.min_frame_gap_ms*1e6,args.max_skew_ms*1e6))
    if len(pairs)<10:
        raise ValueError('Too few synchronized pairs')
    args.output.mkdir(parents=True)
    times=np.array([(v[0]-pairs[0][0])/1e9 for v in pairs])
    fps=float(1/np.median(np.diff(times)))
    baseline=write_settings(args.output,models,fps)
    with ExitStack() as context:
        sources=[]
        for side in ('left','right'):
            sources.append(context.enter_context((args.recording/f'{side}.mjpeg').open('rb')))
            (args.output/side).mkdir()
        manifest=context.enter_context((args.output/'pairs.tsv').open('w'))
        for i,(_,_,*rows) in enumerate(pairs):
            name=f'{i:05d}.png'
            for file,row,grid,side in zip(sources,rows,maps,('left','right')):
                file.seek(row['offset'])
                raw=cv2.imdecode(np.frombuffer(file.read(row['length']),np.uint8),cv2.IMREAD_COLOR)
                if raw is None or raw.shape[:2]!=(480,640):
                    raise ValueError(f'Invalid raw frame {i} on {side}')
                if not cv2.imwrite(str(args.output/side/name),cv2.remap(raw,*grid,cv2.INTER_LINEAR),
                                   [cv2.IMWRITE_PNG_COMPRESSION,1]):
                    raise OSError(f'Could not write {name}')
            print(f'{times[i]:.9f}\t{name}',file=manifest)
    skews=np.array([row[1]/1e6 for row in pairs])
    info=dict(recording=str(args.recording),calibration=str(args.calibration),baseline_m=baseline,
              pairs=len(pairs),max_accepted_skew_ms=args.max_skew_ms,median_skew_ms=float(np.median(skews)),
              p95_skew_ms=float(np.percentile(skews,95)),duration_s=float(times[-1]),median_input_fps=fps)
    (args.output/'sequence.json').write_text(json.dumps(info,indent=2)+'\n')
    print(json.dumps(info,indent=2))


if __name__=='__main__':
    main()
