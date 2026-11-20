#!/usr/bin/env python3
"""Read-only summary of a stereo odometry log's recent processing window."""
import argparse
import json
from pathlib import Path
import re
import statistics


def summarize(path,window=60):
    text=Path(path).read_text(errors='replace')
    rows=[]
    for line in text.splitlines():
        match=re.search(r'\[(\d+\.\d+)\].*Odom: quality=(\d+).*update time=([\d.]+)s delay=([\d.]+)s',line)
        if match:
            rows.append(tuple(map(float,match.groups())))
    if not rows or rows[-1][0]-rows[0][0]<=10:
        raise ValueError('오도메트리 로그가 10초 이상 필요합니다')
    cutoff=max(rows[0][0]+10,rows[-1][0]-window)
    selected=[row for row in rows if row[0]>=cutoff]
    drop_count=0
    for line in text.splitlines():
        if 'Dropping image/scan data' in line:
            match=re.search(r'\[(\d+\.\d+)\]',line)
            if match and cutoff<=float(match[1])<=rows[-1][0]:
                drop_count+=1
    delay=sorted(row[3]*1000 for row in selected)
    duration=rows[-1][0]-cutoff
    zero=sum(row[1]==0 for row in selected)
    return dict(file=str(Path(path).resolve()),window_seconds=round(duration,3),
                samples=len(selected),processed_fps=round(len(selected)/duration,2),
                quality_zero_count=zero,quality_zero_percent=round(zero/len(selected)*100,2),
                drop_warnings=drop_count,median_delay_ms=round(statistics.median(delay),2),
                p95_delay_ms=round(delay[round((len(delay)-1)*.95)],2),
                median_processing_ms=round(statistics.median(row[2] for row in selected)*1000,2),
                errors_total=len(re.findall(r'\bERROR\b',text)),
                note='Observational comparison; scene and motion are not controlled.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log',type=Path)
    parser.add_argument('--window',type=float,default=60)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.window<=0:parser.error('window must be positive')
    report=summarize(args.log,args.window)
    payload=json.dumps(report,indent=2)+'\n'
    print(payload,end='')
    if args.output:
        args.output.write_text(payload)
