#!/usr/bin/env python3
"""Extract sampled white-panel edges, simplify observed runs, and export a wall mesh."""
import argparse
from bisect import bisect_left
from collections import defaultdict
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from map_wall_edges import CAMERA_HEIGHT_M, NOMINAL_PITCH_DEG, PANEL_BOTTOM, PANEL_TOP, extract, geometry, mesh
from jetbot_stereo.wall_tracks import line_model, match_score


def observed_runs(by_bin):
    bins = sorted(index for index, entries in by_bin.items() if len(entries) >= 3)
    if not bins:
        return
    for indices in np.split(bins, np.flatnonzero(np.diff(bins) > 1) + 1):
        if len(indices) < 5:
            continue
        points = []
        for index in indices:
            # Frames contribute equally even if one frame samples a denser part.
            representatives = [np.median(samples, axis=0) for samples in by_bin[index].values()]
            points.append(np.median(representatives, axis=0))
        yield np.asarray(points)


def write_mesh(output, polylines):
    edges = [{'xy_m':[a,b]} for curve in polylines for a,b in zip(curve[:-1],curve[1:])]
    vertices, faces = mesh(edges)
    if not faces:
        raise ValueError('No confirmed white-panel edges were found')
    with (output/'wall-mesh.txt').open('w') as stream:
        print(len(vertices),len(faces),file=stream)
        for v in vertices:
            print(*v,file=stream)
        for f in faces:
            print(*f,file=stream)
    with (output/'wall-mesh.ply').open('w') as stream:
        print(f'ply\nformat ascii 1.0\nelement vertex {len(vertices)}\n'
              'property float x\nproperty float y\nproperty float z\n'
              f'element face {len(faces)}\nproperty list uchar int vertex_indices\nend_header',file=stream)
        for v in vertices:
            print(*v,file=stream)
        for f in faces:
            print(3,*f,file=stream)
    return len(vertices),len(faces)


def preview(output, regions):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax=plt.subplots(figsize=(7.4,5.2),layout='constrained')
    for i,(_,points,simplified) in enumerate(regions):
        ax.scatter(*points.T,s=11,c='#99aab1',label='Repeated edge samples' if i==0 else None)
        ax.plot(*simplified.T,marker='o',markersize=3,color='#a64227',lw=2,
                label='Douglas–Peucker wall lines' if i==0 else None)
    ax.set(xlabel='Local X (m)',ylabel='Local Y (m)',aspect='equal',title='White wall boundaries')
    ax.grid(alpha=.25)
    ax.legend()
    fig.savefig(output/'wall-lines-preview.png',dpi=145)
    plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sequence',type=Path)
    parser.add_argument('orb_result',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--epsilon-m',type=float,default=.02)
    args=parser.parse_args()
    if not 0 < args.epsilon_m <= .1:
        parser.error('epsilon must be positive and at most 0.1 m')
    args.output.mkdir(parents=True)
    cv2.setNumThreads(2)
    prior=json.loads((args.orb_result/'wall-lines.json').read_text())
    basis=np.array(prior['axis_basis_in_orb_world'])
    origin=np.array(prior['origin_in_orb_world_m'])
    tracks={wall['id']:np.asarray(wall['endpoints_xy_m']) for wall in prior['walls']}
    settings=cv2.FileStorage(str(args.sequence/'settings.yaml'),cv2.FILE_STORAGE_READ)
    fx,fy,cx,cy,bf=[settings.getNode('Camera.'+key).real() for key in ('fx','fy','cx','cy','bf')]
    settings.release()
    camera=(fx,fy,cx,cy,bf/fx)
    trajectory=np.loadtxt(args.orb_result/'trajectory.tum',ndmin=2)
    frame_table=[(float(t),name) for t,name in (line.split() for line in (args.sequence/'pairs.tsv').read_text().splitlines())]
    times=[row[0] for row in frame_table]
    rotations=Rotation.from_quat(trajectory[:,4:8]).as_matrix()
    detector=cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco,'ArucoDetector'):
        aruco=cv2.aruco.ArucoDetector(dictionary)
        def marker_points(image):
            return aruco.detectMarkers(image)[0]
    else:
        def marker_points(image):
            return cv2.aruco.detectMarkers(image,dictionary)[0]
    grid_m=.025
    accepted=defaultdict(lambda:defaultdict(lambda:defaultdict(list)))
    inputs=0
    with (args.output/'observed-wall-points.csv').open('w') as points_file:
        print('frame,wall_id,x_m,y_m,z_m',file=points_file)
        for frame_number in range(0,len(trajectory),3):
            time=trajectory[frame_number,0]
            index=bisect_left(times,time)
            nearest=min(range(max(0,index-1),min(len(times),index+1)),key=lambda i:abs(times[i]-time))
            if abs(times[nearest]-time)>1.1e-6:
                continue
            name=frame_table[nearest][1]
            imgs=[cv2.imread(str(args.sequence/side/name)) for side in ('left','right')]
            if any(img is None for img in imgs):
                raise OSError(f'Missing stereo frame {name}')
            for pixels,xy in extract(*imgs,camera,(CAMERA_HEIGHT_M,np.radians(NOMINAL_PITCH_DEG)),
                                     detector,marker_points,dense=True):
                points,_=geometry(pixels,xy,fy,cy)
                world=(points@rotations[frame_number].T+trajectory[frame_number,1:4]-origin)@basis
                if np.max(abs(world[:,2]-PANEL_BOTTOM))>=.075:
                    continue
                endpoints=world[[0,-1],:2]
                scores=[(score,identifier) for identifier,wall in tracks.items()
                        if (score:=match_score(wall,endpoints,angle_deg=12.,distance=.12,gap=.10)) is not None]
                if not scores:
                    continue
                _,identifier=min(scores)
                direction,_,_=line_model(tracks[identifier])
                for point,bin_id in zip(world,np.floor(world[:,:2]@direction/grid_m).astype(int)):
                    accepted[identifier][int(bin_id)][name].append(point[:2])
                    print(name,identifier,*point,sep=',',file=points_file)
                    inputs+=1
    regions=[]
    for identifier, bins in sorted(accepted.items()):
        for points in observed_runs(bins):
            simple=cv2.approxPolyDP(points.astype(np.float32),args.epsilon_m,False).reshape(-1,2)
            regions.append((identifier,points,simple))
    vertices,faces=write_mesh(args.output,[r[2] for r in regions])
    preview(args.output,regions)
    report=dict(coordinate_frame=prior['coordinate_frame'],source='white panel edges reprojected through optimized ORB-SLAM2 poses',
                epsilon_m=args.epsilon_m,continuity_bin_m=grid_m,min_frames_per_bin=3,
                boundary_samples=inputs,polyline_points_before=sum(len(r[1]) for r in regions),
                polyline_vertices_after=sum(len(r[2]) for r in regions),
                polyline_count=len(regions),segments=sum(len(r[2])-1 for r in regions),
                mesh_vertices=vertices,mesh_triangles=faces,panel_height_m=PANEL_TOP-PANEL_BOTTOM,
                panel_bottom_m=PANEL_BOTTOM,panel_thickness_m=.003,
                polylines=[dict(track_id=i,before=points.tolist(),after=simple.tolist()) for i,points,simple in regions])
    (args.output/'wall-lines.json').write_text(json.dumps(report,indent=2)+'\n')
    print({k:v for k,v in report.items() if k!='polylines'})


if __name__=='__main__':
    main()
