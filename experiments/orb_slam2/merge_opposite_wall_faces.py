#!/usr/bin/env python3
"""Build one thin panel with ID 22 and ID 23 on its opposite faces."""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from live_white_walls import Mapper, CAMERA_HEIGHT, PANEL_BASE, PANEL_HEIGHT, recording_frames, mesh, extract
from mark_rtabmap_panel import marker_bitmap


def marker22_placement(args, mapper):
    estimates=[]
    for pair,_,_ in recording_frames(args.recording,.035):
        images=[cv2.remap(im,*grid,cv2.INTER_LINEAR) for im,grid in zip(pair,mapper.maps)]
        tags=[mapper.tags(im) for im in images]
        pose=mapper.pose(*tags)
        if not pose or pose['marker_id']!=22:
            continue
        points=tags[0][22]
        u,v=np.mean(points,axis=0)
        proposals=extract(*images,mapper.camera,(CAMERA_HEIGHT,np.radians(3.5)),mapper.lines,
                          lambda _:list(tags[0].values()),dense=True,panel_height=PANEL_HEIGHT,
                          panel_bottom=PANEL_BASE,right_edge_radius=5)
        left=[pixels for pixels,_ in proposals if pixels[-1,0]<u]
        right=[pixels for pixels,_ in proposals if pixels[0,0]>u]
        if not left or not right:
            continue
        a=max(left,key=lambda p:p[-1,0])[-1]
        b=min(right,key=lambda p:p[0,0])[0]
        alpha=(u-a[0])/(b[0]-a[0])
        top,bottom=(1-alpha)*a[1:]+alpha*b[1:]
        z=PANEL_BASE+PANEL_HEIGHT*(bottom-v)/(bottom-top)
        marker_axis=(points[1]-points[0]+points[2]-points[3])/2
        angle=math.degrees(math.atan2(marker_axis[1],marker_axis[0])-math.atan2(b[1]-a[1],b[0]-a[0]))
        if PANEL_BASE+.05 < z < PANEL_BASE+PANEL_HEIGHT-.05:
            estimates.append((z,angle))
    if len(estimates)<3:
        raise ValueError('Insufficient stable marker 22 observations to estimate its height')
    z,angle=np.median(estimates,axis=0)
    return float(z),float(angle),len(estimates)


def reference_center_x(reference,y,near_x):
    col0,row = np.floor(((np.array([near_x,y])-reference.origin)@reference.rotation)/reference.scale).astype(int)
    pixels=reference.grid[reference.grid.shape[0]-1-row]
    dark=np.flatnonzero(pixels < 50)
    intervals=np.split(dark,np.flatnonzero(np.diff(dark)>1)+1)
    wall = next((i for i in intervals if col0 in i),None)
    if wall is None or not 1 <= len(wall) <= 4:
        raise ValueError('The identified partition has no isolated footprint in the reference map')
    return float(reference.origin[0]+reference.scale*(np.mean(wall)+.5))


def add_pattern(vertices, faces, colors, marker, side, center, normal, clockwise):
    bitmap=marker_bitmap(marker)
    angle=math.radians(-clockwise)
    cos,sin=math.cos(angle),math.sin(angle)
    direction=np.array([-normal[1],normal[0]])
    step=side/6
    def position(u,v):
        xy=np.array(center[:2])+(cos*u-sin*v)*direction
        return (*xy,center[2]+sin*u+cos*v)
    for row in range(6):
        for col in range(6):
            a,b=col*step-side/2,(col+1)*step-side/2
            high,low=side/2-row*step,side/2-(row+1)*step
            start=len(vertices)
            vertices.extend([position(a,low),position(b,low),position(b,high),position(a,high)])
            shade=235 if bitmap[row,col] else 20
            colors.extend([(shade,)*3]*4)
            faces.extend([(start,start+1,start+2),(start,start+2,start+3)])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('output','view22','view23','placement23','recording','calibration','reference'):
        parser.add_argument(name,type=Path)
    args=parser.parse_args()
    cv2.setNumThreads(2)
    mapper=Mapper(args.calibration,args.reference)
    result22,result23=[json.loads(p.read_text()) for p in (args.view22,args.view23)]
    if any(r['frame_id']!='map' or r['pose']['marker_id']!=i for r,i in ((result22,22),(result23,23))):
        raise ValueError('Views must be localized by their respective markers')
    mark23=json.loads(args.placement23.read_text())
    traces=[[xy for line in d['lines_xy_m'] for xy in line] for d in (result22,result23)]
    if any(not trace for trace in traces):
        raise ValueError('Both sides need observed wall edges')
    z22,angle22,N=marker22_placement(args,mapper)
    mid=float(np.median([p[0] for trace in traces for p in trace]))
    x=reference_center_x(mapper.reference,.05,mid)
    for trace in traces:
        if max(abs(pt[0]-x) for pt in trace)>.12:
            raise ValueError('Observed lines do not align with the identified common partition')
    ymin=min(pt[1] for trace in traces for pt in trace)
    marker_center_23=mark23['center_map_xyz_m']
    # The measured 44 cm places the left edge when viewed from the ID23 side.
    # Trace endpoints on the other face support a longer range, but have a
    # different provisional anchor. Keep the surveyed end from the same face.
    ymax=marker_center_23[1]+mark23['center_right_of_wall_left_end_m']
    if ymax-ymin < .25:
        raise ValueError('Insufficient observed wall length')
    thickness=.003
    vertices,faces=mesh([dict(xy_m=[[x,ymin],[x,ymax]])],
                        panel_bottom=PANEL_BASE,panel_top=PANEL_BASE+PANEL_HEIGHT,thickness=thickness)
    colors=[(239,239,237)]*len(vertices)
    meta = {}
    for id,sign,height,angle in ((22,1,z22,angle22),(23,-1,marker_center_23[2],mark23['marker_clockwise_rotation_deg'])):
        anchor=mapper.reference.anchors[id]
        side=anchor['side_mm']/1000.
        y=anchor['y_m'] if id==22 else marker_center_23[1]
        if not ymin+side/2 < y < ymax-side/2:
            raise ValueError(f'ID {id} falls outside the reconstructed panel span')
        face_x=x+sign*thickness/2
        add_pattern(vertices,faces,colors,id,side,[face_x+sign*.0004,y,height],np.array([float(sign),0.]),angle)
        meta[id]=dict(center_map_xyz_m=[face_x,y,height],side_m=side,normal_xy=[float(sign),0.],
                      clockwise_deg=angle,center_height_source='approximate from image position within panel height' if id==22 else 'tape survey')
    args.output.mkdir(exist_ok=True)
    with (args.output/'single-wall-two-markers-mesh.txt').open('w') as stream:
        print(len(vertices),len(faces),file=stream)
        for v,c in zip(vertices,colors):
            print(*v,*c,file=stream)
        for f in faces:
            print(*f,file=stream)
    report=dict(coordinate_frame='map',panels=1,axis_x_m=x,y_endpoints_m=[ymin,ymax],
                physical_thickness_m=thickness,wall_height_m=PANEL_HEIGHT,
                wall_base_above_floor_m=PANEL_BASE,markers=meta,id22_height_observations=N,
                surveyed_id23_left_edge_distance_m=mark23['center_right_of_wall_left_end_m'],
                sample_face_x_medians=[float(np.median([p[0] for p in trace])) for trace in traces],
                absolute_anchor_positions_approximate=True)
    (args.output/'single-wall-two-markers.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
