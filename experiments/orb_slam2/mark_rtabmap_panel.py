#!/usr/bin/env python3
"""Place the surveyed ID 23 target on a wall reconstructed in the same capture."""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

from live_white_walls import Mapper, recording_frames, mesh
from prepare_stereo import camera_models


def aligned_panel(result, anchor, survey):
    yaw = math.radians(anchor['normal_yaw_deg'])
    reference_normal = np.array([math.cos(yaw), math.sin(yaw)])
    marker_xy = np.array([anchor['x_m'],anchor['y_m']])
    candidates = []
    for index, polyline in enumerate(result['lines_xy_m']):
        line = np.asarray(polyline)
        for a,b in zip(line,line[1:]):
            direction = b-a
            length = np.linalg.norm(direction)
            along = direction/length
            if np.dot(along, [-reference_normal[1],reference_normal[0]]) < 0:
                a,b = b,a
                along = -along
            normal = np.array([along[1],-along[0]])
            extent = float((marker_xy-a)@along)
            offset = abs(float((marker_xy-a)@normal))
            if normal @ reference_normal < .96 or offset > .07 or not 0.07 < extent < length-.07:
                continue
            candidates.append((offset,index,a,b,along,normal,extent))
    if not candidates:
        raise ValueError('No observed wall segment contains the detected ID 23')
    _,index,a,b,along,normal,extent = min(candidates,key=lambda v:v[0])
    center = a+extent*along
    offset_left = survey['center_right_of_panel_left_end_m']
    measured_left = center-offset_left*along
    return index,a,b,along,normal,center,measured_left,extent


def marker_bitmap(tag):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco,'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary,tag,6)
    return cv2.aruco.drawMarker(dictionary,tag,6)


def wall_with_marker(result, anchor, survey, roll=0.):
    index,a,b,along,normal,center,left,observed_left = aligned_panel(result,anchor,survey)
    top = float(np.mean(survey['floor_to_white_panel_top_m_range']))
    bottom = top-float(np.mean(survey['white_panel_height_m_range']))
    z = top-survey['center_below_white_panel_top_m']
    side = survey['marker_black_square_side_m']
    vertical_half = side/2 * (abs(math.cos(roll)) + abs(math.sin(roll)))
    if not bottom < z-vertical_half < z+vertical_half < top:
        raise ValueError('Marker height is inconsistent with the measured panel')
    # Use the surveyed left extent when it is close to the observed span.
    # This is a measured edge position, not an extra detected image sample.
    extended = observed_left < survey['center_right_of_panel_left_end_m']
    adjust = extended and np.linalg.norm(a-left) <= .18 and len(result['lines_xy_m'][index]) == 2
    observed = [np.asarray(curve) for curve in result['lines_xy_m']]
    if adjust:
        nearest = 0 if np.linalg.norm(observed[index][0]-a) < np.linalg.norm(observed[index][-1]-a) else -1
        observed[index][nearest] = left
    edges = [{'xy_m':[a0,b0]} for line in observed for a0,b0 in zip(line,line[1:])]
    vertices,faces = mesh(edges,panel_bottom=bottom,panel_top=top)
    colors = [(239,239,237)]*len(vertices)
    bitmap = marker_bitmap(survey['marker_id'])
    # Offset the printed face just outside the wall's 1.5 mm half-thickness.
    surface = center+.0019*normal
    step=side/6.
    cos_roll, sin_roll = math.cos(roll), math.sin(roll)

    def vertex(horizontal, vertical):
        xy = surface + (cos_roll*horizontal-sin_roll*vertical)*along
        return (*xy, z + sin_roll*horizontal+cos_roll*vertical)

    for row in range(6):
        for col in range(6):
            lo,hi=-side/2+col*step,-side/2+(col+1)*step
            high=side/2-row*step
            low=high-step
            first=len(vertices)
            vertices.extend([vertex(lo,low),vertex(hi,low),vertex(hi,high),vertex(lo,high)])
            level = 235 if bitmap[row,col] else 20
            colors.extend([(level,)*3]*4)
            faces.extend([(first,first+1,first+2),(first,first+2,first+3)])
    meta = dict(marker_id=23,center_map_xyz_m=[*center.tolist(),z],wall_left_end_from_survey_map_xy_m=left.tolist(),
                marker_side_m=side,marker_anchor_xy_m=[anchor['x_m'],anchor['y_m']],
                center_below_wall_top_m=survey['center_below_white_panel_top_m'],
                center_right_of_wall_left_end_m=survey['center_right_of_panel_left_end_m'],
                observed_span_left_edge_to_center_m=observed_left,left_extent_extended_using_tape=bool(adjust),
                marker_clockwise_rotation_deg=-math.degrees(roll),coordinate_frame=result['frame_id'])
    return vertices,faces,colors,meta


def show_projection(args, mapper, result, survey):
    pair = next((images for images,stamp,_ in recording_frames(args.recording,.03)
                 if stamp == result['stamp_s']),None)
    if pair is None:
        raise ValueError('Could not load the same pair used for the mesh')
    rect = [cv2.remap(im,*grid,cv2.INTER_LINEAR) for im,grid in zip(pair,mapper.maps)]
    marker = survey['marker_id']
    h = survey['marker_black_square_side_m']/2
    shape = h*np.array([[-1,1,0],[1,1,0],[1,-1,0],[-1,-1,0]],dtype=float)
    top = survey['center_below_white_panel_top_m']
    bottom = top-float(np.mean(survey['white_panel_height_m_range']))
    left = -survey['center_right_of_panel_left_end_m']
    sample = np.array([[0.,0,0],[left,top,0],[h,top,0],[left,bottom,0],[h,bottom,0]])
    predictions = {}
    for side,im in zip(('left','right'),rect):
        tags=mapper.tags(im)
        if marker not in tags:
            continue
        ok,rotation,translation=cv2.solvePnP(shape,tags[marker],mapper.k,None,
                                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:continue
        xy=cv2.projectPoints(sample,rotation,translation,mapper.k,None)[0].reshape(-1,2)
        predictions[side]=xy.tolist()
        q=np.rint(xy).astype(int)
        for a,b,color in [(1,2,(0,205,255)),(3,4,(45,220,45)),(1,3,(245,195,40))]:
            cv2.line(im,tuple(q[a]),tuple(q[b]),color,2,cv2.LINE_AA)
        cv2.circle(im,tuple(q[0]),5,(40,40,240),2)
    cv2.imwrite(str(args.output/'survey-projection.jpg'),np.hstack(rect))
    return predictions



def measured_boundaries(args, mapper, result, survey):
    pair = next((images for images,stamp,_ in recording_frames(args.recording,.03)
                 if stamp == result['stamp_s']),None)
    overlay = cv2.imread(str(args.boundary_image))
    old_preview = cv2.imread(str(args.output/'survey-projection.jpg'))
    if pair is None or overlay is None or old_preview is None or overlay.shape != old_preview.shape:
        raise ValueError('Could not read the stereo pair and its matching boundary overlay')
    rect = [cv2.remap(im,*grid,cv2.INTER_LINEAR) for im,grid in zip(pair,mapper.maps)]
    tag = survey['marker_id']
    corners = [mapper.tags(im).get(tag) for im in rect]
    if any(c is None for c in corners):
        raise ValueError('ID 23 is missing from one rectified frame')
    projections = [c[3] for c in camera_models(args.calibration)]
    matches = cv2.triangulatePoints(*projections,corners[0].T,corners[1].T)
    points = (matches[:3]/matches[3]).T
    right = (points[1]-points[0]+points[2]-points[3])/2
    up = (points[0]-points[3]+points[1]-points[2])/2
    normal = np.cross(right,up)
    normal /= np.linalg.norm(normal)
    angles = []
    records = {}
    for label,actual,prior,image,projection in zip(('left','right'),np.hsplit(overlay,2),
                                                  np.hsplit(old_preview,2),rect,projections):
        color = actual.astype(np.int16)
        base = prior.astype(np.int16)
        chroma = color[:,:,2]-color[:,:,:2].max(axis=2)
        old_chroma = base[:,:,2]-base[:,:,:2].max(axis=2)
        annotation = (chroma>35) & (chroma-old_chroma>22)
        records[label] = {}
        for edge,(y0,y1),line_color in [('upper',(114,141),(0,205,255)),('lower',(260,283),(45,220,45))]:
            y,x = np.nonzero(annotation[y0:y1])
            y += y0
            if len(x)<100 or np.ptp(x)<200:
                raise ValueError(f'Could not identify the red {label} {edge} line')
            slope,intercept = np.polyfit(x,y,1)
            residual = np.percentile(abs(y-(slope*x+intercept)),95)
            if residual>3.:
                raise ValueError(f'Red {label} {edge} line is not coherent')
            points2d=np.array([[x.min(), slope*x.min()+intercept],[x.max(),slope*x.max()+intercept]])
            cv2.line(image,*(tuple(v) for v in np.rint(points2d).astype(int)),line_color,2,cv2.LINE_AA)
            ray_plane = projection[:,:3].T @ np.array([slope,-1.,intercept])
            along = np.cross(normal,ray_plane)
            along *= np.sign(along[0])/np.linalg.norm(along)
            wall_up = np.cross(normal,along)
            angle = math.atan2(float(right@wall_up),float(right@along))
            angles.append(angle)
            records[label][edge] = dict(endpoints_px=points2d.tolist(),annotation_p95_residual_px=float(residual),
                                        marker_clockwise_rotation_deg=-math.degrees(angle))
        cv2.polylines(image,[np.rint(corners[0 if label=='left' else 1]).astype(np.int32)],True,
                      (215,82,235),2,cv2.LINE_AA)
    if not cv2.imwrite(str(args.output/'measured-boundaries-preview.jpg'),np.hstack(rect)):
        raise OSError('Could not save the corrected preview')
    return float(np.median(angles)), records

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path)
    p.add_argument('recording',type=Path)
    p.add_argument('calibration',type=Path)
    p.add_argument('reference',type=Path)
    p.add_argument('--boundary-image',type=Path,help='Red top and bottom annotations on the rectified preview')
    args=p.parse_args()
    cv2.setNumThreads(2)
    survey=json.loads((args.output/'id23-wall-survey.json').read_text())
    result=json.loads((args.output/'snapshot.json').read_text())
    if result['frame_id'] != 'map' or result['pose']['marker_id'] != 23:
        raise SystemExit('Use a current capture whose pose was established by ID 23')
    mapper=Mapper(args.calibration,args.reference)
    roll,boundaries = measured_boundaries(args,mapper,result,survey) if args.boundary_image else (0.,None)
    vertices,faces,colors,meta=wall_with_marker(result,mapper.reference.anchors[23],survey,roll)
    if boundaries:
        meta['image_boundary_annotations'] = boundaries
    else:
        meta['survey_projection_px']=show_projection(args,mapper,result,survey)
    suffix = '-measured' if boundaries else ''
    with (args.output/f'white-wall-id23-mesh{suffix}.txt').open('w') as stream:
        print(len(vertices),len(faces),file=stream)
        for vertex,color in zip(vertices,colors):
            print(*vertex,*color,file=stream)
        for face in faces:
            print(*face,file=stream)
    (args.output/f'id23-placement{suffix}.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta,indent=2))


if __name__=='__main__':
    main()
