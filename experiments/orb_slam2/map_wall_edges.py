#!/usr/bin/env python3
"""Fuse visible white-panel boundaries into a local ORB-SLAM2 map."""
import argparse
import csv
import json
from pathlib import Path
from bisect import bisect_left
import sys

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

PROJECT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(PROJECT/'scripts'), str(PROJECT/'src/jetbot_stereo')]
from reconstruct_white_partitions import extract, PANEL_BOTTOM, PANEL_TOP, mesh
from jetbot_stereo.wall_tracks import WallTracks

CAMERA_HEIGHT_M=.099
NOMINAL_PITCH_DEG=3.5


def unit(vector):
    return vector/np.linalg.norm(vector)


def geometry(pixels, xy, fy, cy):
    """Recover the original rectified optical coordinates of the modeled base edge."""
    fraction=(CAMERA_HEIGHT_M-PANEL_BOTTOM)/(PANEL_TOP-PANEL_BOTTOM)
    horizon=np.mean(pixels[:,2] + fraction*(pixels[:,1]-pixels[:,2]))
    pitch=np.arctan2(cy-horizon,fy)
    sine,cosine=np.sin(pitch),np.cos(pitch)
    forward,left=xy.T
    z=PANEL_BOTTOM-CAMERA_HEIGHT_M
    points=np.column_stack((-left,-sine*forward-cosine*z,cosine*forward-sine*z))
    return points,np.array([0.,-cosine,-sine])


def write_ply(output, walls):
    vertices,faces=mesh([{'xy_m':wall['endpoints_xy_m']} for wall in walls])
    with output.open('w') as stream:
        print('ply\nformat ascii 1.0',file=stream)
        print(f'element vertex {len(vertices)}\nproperty float x\nproperty float y\nproperty float z',file=stream)
        print(f'element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header',file=stream)
        for vertex in vertices:
            print(*vertex,file=stream)
        for face in faces:
            print(3,*face,file=stream)


def preview_map(output, walls, track, sparse, pixels, times):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax=plt.subplots(figsize=(7.8,5.8), layout='constrained')
    if sparse.size:
        ax.scatter(*sparse[:,:2].T,s=1,c='#a0a5a9',alpha=.32,label='ORB points')
    for index, section in enumerate(np.split(track, np.flatnonzero(np.diff(times) > .45) + 1)):
        ax.plot(*section[:,:2].T, linewidth=1.2, color='#316970', marker='.', markersize=1.4,
                label='Camera trajectory' if index == 0 else None)
    for i,wall in enumerate(walls):
        a,b=np.array(wall['endpoints_xy_m'])
        ax.plot([a[0],b[0]],[a[1],b[1]],linewidth=2.5,color='#ad5334',label='Observed wall edges' if i==0 else None)
        center=(a+b)*.5
        ax.annotate(str(wall['id']),center,xytext=(0,4),textcoords='offset points',fontsize=7)
    ax.scatter(0,0,c='#202931',marker='+',s=70,zorder=5,label='Local origin')
    ax.set(xlabel='Local X (m)',ylabel='Local Y (m)',title='ORB-SLAM2 and observed wall edges',aspect='equal')
    ax.grid(alpha=.22)
    ax.legend(loc='best')
    fig.savefig(output/'wall-map.png',dpi=145)
    plt.close(fig)
    if pixels is not None:
        cv2.imwrite(str(output/'image-wall-observations.jpg'),pixels)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sequence',type=Path)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    cv2.setNumThreads(2)
    settings=cv2.FileStorage(str(args.sequence/'settings.yaml'),cv2.FILE_STORAGE_READ)
    fx,fy,cx,cy,bf=[settings.getNode('Camera.'+k).real() for k in ('fx','fy','cx','cy','bf')]
    settings.release()
    if min(fx,fy,bf)<=0:
        raise ValueError('Missing rectified camera geometry')
    camera=(fx,fy,cx,cy,bf/fx)
    poses=np.loadtxt(args.output/'trajectory.tum',ndmin=2)
    if poses.shape[1]!=8:
        raise ValueError('Expected a TUM trajectory')
    state={float(r['timestamp']):int(r['state'])
           for r in csv.DictReader((args.output/'tracking.tsv').open(),delimiter='\t')}
    frames=[(float(t),name) for t,name in (line.split() for line in (args.sequence/'pairs.tsv').read_text().splitlines())]
    alignment=Rotation.from_quat(poses[:,4:8]).as_matrix()
    times = poses[:,0].tolist()
    detection=cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco,'ArucoDetector'):
        aruco=cv2.aruco.ArucoDetector(dictionary)
        def tags(image):
            return aruco.detectMarkers(image)[0]
    else:
        def tags(image):
            return cv2.aruco.detectMarkers(image,dictionary)[0]
    observations=[]
    directions=[]
    preview=None
    for time,name in frames[::3]:
        if state.get(time) != 2:
            continue
        pos = bisect_left(times, time)
        idx = min(range(max(0, pos-1), min(len(times), pos+1)), key=lambda i: abs(times[i]-time))
        if abs(times[idx]-time) > 1.1e-6:
            continue
        left,right=(cv2.imread(str(args.sequence/side/name)) for side in ('left','right'))
        matches=extract(left,right,camera,(CAMERA_HEIGHT_M,np.radians(NOMINAL_PITCH_DEG)),detection,tags)
        if not matches:
            continue
        rotation,translation=alignment[idx],poses[idx,1:4]
        geo=[]
        for pixels,xy in matches:
            points,up=geometry(pixels,xy,fy,cy)
            geo.append(points@rotation.T+translation)
            directions.append(rotation@up)
        if preview is None:
            preview=left.copy()
            for pixels,_ in matches:
                for edge,color in ((1,(0,200,255)),(2,(45,220,50))):
                    ends=np.rint(pixels[:,[0,edge]]).astype(int)
                    cv2.line(preview,*[tuple(p) for p in ends],color,2)
        observations.append((time,geo))
    if directions:
        up=unit(np.median(directions,axis=0))
    else:
        up=alignment[0]@np.array([0.,-1.,0.])
    forward=alignment[0][:,2]
    x=unit(forward-(up@forward)*up)
    basis=np.column_stack((x,np.cross(up,x),up))
    origin=poses[0,1:4]-CAMERA_HEIGHT_M*up
    tracks=WallTracks(max_tracks=256,min_observations=5)
    for stamp, candidates in observations:
        values=[]
        for xyz in candidates:
            projected=(xyz-origin)@basis
            if np.max(abs(projected[:,2]-PANEL_BOTTOM))>=.075:
                continue
            values.append(dict(endpoints_xy_m=projected[:,:2],height_m=PANEL_TOP-PANEL_BOTTOM,
                               floor_z_m=PANEL_BOTTOM))
        tracks.observe(values,stamp)
    walls=tracks.confirmed()
    points=np.loadtxt(args.output/'map-points.xyz',ndmin=2)
    points=(points-origin)@basis if points.size else np.empty((0,3))
    track=(poses[:,1:4]-origin)@basis
    # Limit the figure to nearby sparse points; preserve the full upstream export.
    lower=track.min(axis=0)[:2]-2.8
    upper=track.max(axis=0)[:2]+2.8
    nearby=(np.all((points[:,:2]>=lower)&(points[:,:2]<=upper),axis=1) & (abs(points[:,2])<1.5))
    with (args.output/'wall-lines.json').open('w') as file:
        json.dump(dict(coordinate_frame='orb_slam2_local_level', camera_pose_source='ORB-SLAM2 optimized TUM',
                       axis_basis_in_orb_world=basis.tolist(),origin_in_orb_world_m=origin.tolist(),
                       panel_height_m=PANEL_TOP-PANEL_BOTTOM, panel_base_m=PANEL_BOTTOM,
                       tracked_frames=sum(value==2 for value in state.values()),input_frames=len(state),
                       used_pose_rows=len(poses),frames_with_detected_edges=len(observations),
                       sparse_points=len(points),walls=walls),file,indent=2)
        file.write('\n')
    write_ply(args.output/'wall-mesh.ply',walls)
    preview_map(args.output,walls,track,points[nearby],preview,poses[:,0])
    print(f'{len(walls)} wall segments from {len(observations)} frames with edges; {len(points)} ORB points')


if __name__=='__main__':
    main()
