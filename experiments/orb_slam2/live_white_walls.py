#!/usr/bin/env python3
"""Extract only the visible white partitions, with ArUco-only localization."""
import argparse
from bisect import bisect_left
from collections import deque
import json
import math
from pathlib import Path
import sys
from time import monotonic

import cv2
import numpy as np
import yaml

from prepare_stereo import camera_models

PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT/'scripts'))
from reconstruct_white_partitions import extract, mesh
sys.path.insert(0, str(PROJECT / "src/jetbot_stereo"))
from jetbot_stereo.wall_tracks import match_score

PANEL_BASE = .005
PANEL_HEIGHT = .1995
CAMERA_HEIGHT = .099
EPSILON = .02
FRAME_TOLERANCE = .030
POSE_TIMEOUT = .75


class Reference:
    def __init__(self, map_file):
        data = yaml.safe_load(map_file.read_text())
        self.grid = cv2.imread(str(map_file.parent/data['image']), cv2.IMREAD_GRAYSCALE)
        if self.grid is None or data.get('negate', 0):
            raise ValueError('Expected an unnegated reference occupancy image')
        self.scale = float(data['resolution'])
        self.origin = np.asarray(data['origin'][:2])
        yaw = data['origin'][2]
        self.rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        occupancy = np.uint8(1-self.grid/255. > data['occupied_thresh'])
        self.free_grid = 1-self.grid/255. < data['free_thresh']
        contours, _ = cv2.findContours(occupancy, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        corners = []
        for contour in contours:
            xy = cv2.approxPolyDP(contour, .035/self.scale, True).reshape(-1,2).astype(float)
            xy[:,1] = self.grid.shape[0]-xy[:,1]-.5
            xy[:,0] += .5
            xy = (xy * self.scale) @ self.rotation.T + self.origin
            for index, corner in enumerate(xy):
                a,b = xy[index-1] - corner, xy[(index+1) % len(xy)] - corner
                an,bn = np.linalg.norm(a),np.linalg.norm(b)
                if min(an,bn) >= .12 and abs(a@b)/(an*bn) < .35:
                    corners.append(corner)
        self.corners = np.asarray(corners).reshape(-1,2)
        self.anchors = {}
        for path in map_file.parent.glob('aruco-id*-anchor.json'):
            anchor = json.loads(path.read_text())
            self.anchors[int(anchor['marker_id'])] = anchor

    def free(self, xy):
        col,row = np.floor(((np.asarray(xy)-self.origin)@self.rotation)/self.scale).astype(int)
        h,w = self.grid.shape
        return 0 <= row < h and 0 <= col < w and bool(self.free_grid[h-1-row,col])

    def snap(self, curves):
        changes = []
        used = set()
        for i, line in enumerate(curves):
            for j in range(i+1,len(curves)):
                other = curves[j]
                a,b = line[-1]-line[0], other[-1]-other[0]
                a,b = a/np.linalg.norm(a), b/np.linalg.norm(b)
                if abs(a@b) > .45:
                    continue
                t,_ = np.linalg.solve(np.column_stack((a,-b)),other[0]-line[0])
                cross = line[0]+t*a
                m = min((0,-1),key=lambda k:np.linalg.norm(line[k]-cross))
                n = min((0,-1),key=lambda k:np.linalg.norm(other[k]-cross))
                if (i,m) in used or (j,n) in used:
                    continue
                if max(np.linalg.norm(line[m]-cross),np.linalg.norm(other[n]-cross)) > .12 or not len(self.corners):
                    continue
                corner = min(self.corners,key=lambda c:np.linalg.norm(c-cross))
                if max(np.linalg.norm(corner-cross),np.linalg.norm(corner-line[m]),
                       np.linalg.norm(corner-other[n])) > .12:
                    continue
                if min(np.linalg.norm(line[-1 if m == 0 else 0]-corner),
                       np.linalg.norm(other[-1 if n == 0 else 0]-corner)) < .08:
                    continue
                line[m],other[n] = corner,corner
                used.update(((i,m),(j,n)))
                changes.append(dict(lines=[i,j],corner_xy_m=corner.tolist()))
        return changes


def connect_panel_spans(proposals, flush_marker, marker_center):
    """Join coplanar spans separated by the surveyed flush marker or a small detection break."""
    groups = []
    joined = []
    for pixels, xy in sorted(proposals, key=lambda row: row[0][0, 0]):
        if groups:
            previous_pixels, previous_xy = groups[-1]
            a, b = previous_xy[-1], xy[0]
            gap = float(np.linalg.norm(b-a))
            footprint_a = previous_xy[[0,-1]]
            footprint_b = xy[[0,-1]]
            plane_match = match_score(footprint_a, footprint_b, angle_deg=7., distance=.045, gap=.025)
            marked_plane_match = (flush_marker is not None and
                                  match_score(footprint_a, footprint_b, angle_deg=10.,
                                              distance=.075, gap=.44) is not None)
            if plane_match is not None or marked_plane_match:
                left_x, right_x = previous_pixels[-1,0], pixels[0,0]
                small_break = plane_match is not None and gap <= .025 and 0 <= right_x-left_x <= 14
                marked_gap = False
                if marked_plane_match and marker_center is not None and .025 < gap <= .44:
                    x0, x1 = flush_marker[:,0].min(), flush_marker[:,0].max()
                    width = x1-x0
                    u = (x0+x1)/2
                    tangent = (footprint_a[1]-footprint_a[0]) / np.linalg.norm(np.diff(footprint_a,axis=0))
                    perpendicular = np.array([-tangent[1],tangent[0]])
                    projected_marker = float(tangent @ (marker_center-a))
                    length = float(tangent @ (b-a))
                    same_face = max(abs(float(perpendicular @ (marker_center-a))), abs(float(perpendicular @ (marker_center-b)))) <= .07
                    overlaps_marker = 0 <= projected_marker <= length
                    covers_image_marker = (left_x <= x0 < x1 <= right_x
                                            and x0-left_x <= width and right_x-x1 <= width)
                    if same_face and overlaps_marker and covers_image_marker:
                        upper_lower = [
                            [np.polyval(np.polyfit(rows[:,0], rows[:,edge], 1), u) for edge in (1,2)]
                            for rows in (previous_pixels, pixels)]
                        disagreement = abs(np.subtract(*upper_lower))
                        marked_gap = bool(np.all(disagreement <= (6.,14.)))
                if small_break or marked_gap:
                    joined.append(dict(kind='flush_id23' if marked_gap else 'small_gap',
                                       gap_m=gap, endpoints_xy_m=[a.tolist(),b.tolist()],
                                       upper_endpoints_px=[previous_pixels[-1,[0,1]].tolist(),
                                                           pixels[0,[0,1]].tolist()]))
                    groups[-1] = (np.vstack((previous_pixels,pixels)),np.vstack((previous_xy,xy)))
                    continue
        groups.append((pixels,xy))
    return groups,joined


def extend_upper_edge(pixels, xy, images, camera, pose, anchor, flush_marker):
    """Follow the visible upper edge on the same plane as the flush ID 23."""
    theta = pose['yaw_rad']
    rotation = np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
    camera_xy = np.asarray(pose['xy_m'])
    local = (xy-camera_xy)@rotation
    center = np.median(local,axis=0)
    _,_,basis = np.linalg.svd(local-center,full_matrices=False)
    normal = basis[-1]
    marker = (np.array([anchor['x_m'],anchor['y_m']])-camera_xy)@rotation
    if np.linalg.norm(local[0]-local[-1]) < .15 or abs(normal@(marker-center)) > .07:
        return xy,[]
    azimuth = math.radians(anchor['normal_yaw_deg'])
    expected = np.array([math.cos(azimuth),math.sin(azimuth)])@rotation
    if abs(normal@expected) < math.cos(math.radians(12)):
        return xy,[]
    fx,fy,cx,cy,baseline = camera
    horizon = np.median(pixels[:,2] + (CAMERA_HEIGHT-PANEL_BASE)/PANEL_HEIGHT*(pixels[:,1]-pixels[:,2]))
    pitch = np.arctan2(cy-horizon,fy)
    sine,cosine = np.sin(pitch),np.cos(pitch)
    height = PANEL_BASE+PANEL_HEIGHT-CAMERA_HEIGHT
    line = np.polyfit(pixels[:,0],pixels[:,1],1)
    labs = [cv2.cvtColor(cv2.medianBlur(im,3),cv2.COLOR_BGR2LAB) for im in images]
    gradients = [cv2.Sobel(lab[:,:,0],cv2.CV_32F,0,1,ksize=3) for lab in labs]
    rows, cols = labs[0].shape[:2]
    marker_center_x = np.mean(flush_marker[:,0])
    paper_min, paper_max = marker_center_x + 1.42*(np.array([flush_marker[:,0].min(),
                                                            flush_marker[:,0].max()])-marker_center_x)

    def edge_at(image_index, u, v):
        grad = gradients[image_index]
        lab = labs[image_index]
        x,y = round(u),round(v)
        if not (3 <= x < cols-3 and 6 <= y < rows-21):
            return None
        profile = abs(grad[y-3:y+4,x])
        actual = y-3+int(np.argmax(profile))
        below = lab[actual+6:actual+18,x].astype(float)
        if max(profile) < 24 or np.median(below[:,0]) < 78:
            return None
        if np.median(np.linalg.norm(below[:,1:]-128,axis=1)) > 20:
            return None
        return actual

    additions = []
    starts,ends = [],[]
    for edge,direction,extension in ((0,-1,starts),(-1,1,ends)):
        reject = 0
        u = round(pixels[edge,0])
        while True:
            u += direction*3
            if not 3 <= u < cols-3 or paper_min <= u <= paper_max:
                break
            alpha = (u-cx)/fx
            denom = normal[0]-normal[1]*alpha*cosine
            if abs(denom) < .10:
                break
            forward = (normal@center-normal[1]*alpha*sine*height)/denom
            depth = cosine*forward-sine*height
            if depth <= .30:
                break
            p = np.array([forward,-alpha*depth])@rotation.T+camera_xy
            if np.linalg.norm(p-xy[edge]) > .25:
                break
            right_x = u - fx*baseline/depth
            y = np.polyval(line,u)
            left_y,right_y = edge_at(0,u,y),edge_at(1,right_x,y)
            if left_y is None or right_y is None or abs(left_y-right_y) > 3:
                reject+=1
                if reject>=3:
                    break
                continue
            reject=0
            extension.append(p)
            additions.append([float(u),float(left_y),float(right_x),float(right_y)])
    pieces = ([np.array(starts[::-1])] if starts else []) + [xy] + ([np.array(ends)] if ends else [])
    return np.vstack(pieces),additions


class Mapper:
    def __init__(self, calibration, map_file):
        self.reference = Reference(map_file)
        models = camera_models(calibration)
        self.maps = [cv2.initUndistortRectifyMap(*c,(640,480),cv2.CV_16SC2) for c in models]
        p,q = models[0][3],models[1][3]
        self.k = p[:,:3].copy()
        self.camera = (p[0,0],p[1,1],p[0,2],p[1,2],(p[0,3]-q[0,3])/p[0,0])
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco,'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detect = cv2.aruco.ArucoDetector(dictionary,params).detectMarkers
        else:
            params = cv2.aruco.DetectorParameters_create()
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detect = lambda image: cv2.aruco.detectMarkers(image,dictionary,parameters=params)
        self.lines = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)

    def tags(self, image):
        corners, ids, _ = self.detect(image)
        return {int(i):p.reshape(4,2) for i,p in zip(ids.ravel() if ids is not None else [],corners)}

    def pose(self, left, right):
        solutions = []
        for marker_id, corners in left.items():
            anchor = self.reference.anchors.get(marker_id)
            if not anchor or marker_id not in right:
                continue
            if (np.max(abs(corners[:,1]-right[marker_id][:,1])) > 3
                    or np.any(corners[:,0]-right[marker_id][:,0] <= 1)
                    or cv2.contourArea(corners) < 450):
                continue
            half = anchor['side_mm']/2000.
            target = half * np.array([[-1,1,0],[1,1,0],[1,-1,0],[-1,-1,0]],dtype=float)
            ok, vector, position = cv2.solvePnP(target,corners,self.k,None,flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok or position[2,0] <= 0:
                continue
            projected = cv2.projectPoints(target,vector,position,self.k,None)[0].reshape(4,2)
            rms = float(np.sqrt(np.mean(np.sum((projected-corners)**2,axis=1))))
            if rms > 1.5:
                continue
            yaw = math.radians(anchor['normal_yaw_deg'])
            normal = np.array([math.cos(yaw), math.sin(yaw), 0.])
            up = np.array([0.,0.,1.])
            from_tag = np.column_stack((np.cross(up,normal),up,normal))
            rotation = from_tag @ cv2.Rodrigues(vector)[0].T
            if rotation[2,1] > -.78 or abs(rotation[2,2]) > .45:
                continue
            center = np.array([anchor['x_m'],anchor['y_m']])-(rotation@position.ravel())[:2]
            if not self.reference.free(center):
                continue
            solutions.append((rms,dict(xy_m=center.tolist(),yaw_rad=math.atan2(rotation[1,2],rotation[0,2]),
                                       marker_id=marker_id,reprojection_rms_px=rms,
                                       anchor_position_std_m=float(anchor.get('position_std_m',.20)))))
        return min(solutions,key=lambda s:s[0])[1] if solutions else None

    def process(self, images, stamp):
        rect = [cv2.remap(im,*grid,cv2.INTER_LINEAR) for im,grid in zip(images,self.maps)]
        left,right = (self.tags(im) for im in rect)
        preview = [im.copy() for im in rect]
        pose = self.pose(left,right)
        proposals = extract(*rect,self.camera,(CAMERA_HEIGHT,np.radians(3.5)),self.lines,
                            lambda _: [p.reshape(1,4,2) for p in left.values()],dense=True,
                            panel_height=PANEL_HEIGHT,panel_bottom=PANEL_BASE,right_edge_radius=5)
        points = []
        curves = []
        visible = []
        for pixels, xy in proposals:
            if pose:
                theta = pose['yaw_rad']
                rotation = np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
                xy = xy@rotation.T + pose['xy_m']
            points.extend(xy.tolist())
            visible.append((pixels,xy))
            for edge,color in ((1,(0,195,255)),(2,(30,225,45))):
                for x,y in np.rint(pixels[:,[0,edge]]).astype(int):
                    cv2.circle(preview[0],(x,y),2,color,-1)
        # ID 23 is known to be flush with this panel. Other tags do not
        # establish that a gap is occupied by wall material.
        flush = left.get(23) if pose and pose['marker_id'] == 23 else None
        anchor = self.reference.anchors.get(23) if flush is not None else None
        marker_center = np.array([anchor['x_m'],anchor['y_m']]) if anchor else None
        groups, joined = connect_panel_spans(visible,flush,marker_center)
        upper_samples = []
        for pixels,xy in groups:
            if anchor is not None:
                xy, samples = extend_upper_edge(pixels,xy,rect,self.camera,pose,anchor,flush)
                upper_samples.extend(samples)
            curve = cv2.approxPolyDP(xy.astype(np.float32),EPSILON,False).reshape(-1,2).astype(float)
            if len(curve) >= 2:
                curves.append(curve)
        for gap in joined:
            if gap['kind'] != 'flush_id23':
                continue
            start,end = np.array(gap['upper_endpoints_px'])
            disparity = np.median(flush[:,0]-right[23][:,0])
            for image,offset in zip(preview,(0.,disparity)):
                a,b = start-[offset,0.],end-[offset,0.]
                for step in np.arange(0.,1.,.055):
                    c = tuple(np.rint(a+(b-a)*step).astype(int))
                    d = tuple(np.rint(a+(b-a)*min(1.,step+.028)).astype(int))
                    cv2.line(image,c,d,(215,79,236),2,cv2.LINE_AA)
        for left_x,left_y,right_x,right_y in upper_samples:
            cv2.circle(preview[0],(round(left_x),round(left_y)),2,(255,207,24),-1)
            cv2.circle(preview[1],(round(right_x),round(right_y)),2,(255,207,24),-1)
        changes = self.reference.snap(curves) if pose else []
        return dict(stamp_s=stamp,frame_id='map' if pose else 'stereo_left_camera_level',
                    tracked=pose is not None,pose=pose,detected_ids=sorted(left.keys()|right.keys()),
                    registered_ids_seen_in_both=sorted(left.keys() & right.keys() & self.reference.anchors.keys()),
                    points_xy_m=points,lines_xy_m=[curve.tolist() for curve in curves],
                    snapped_corners=changes,joined_gaps=joined,upper_edge_extra_samples_px=upper_samples,
                    panel_base_m=PANEL_BASE,panel_height_m=PANEL_HEIGHT), np.hstack(preview)


def decode(message):
    image = cv2.imdecode(np.frombuffer(message,np.uint8),cv2.IMREAD_COLOR)
    if image is None or image.shape[:2] != (480,640):
        raise ValueError('Expected a compressed 640x480 camera frame')
    return image


def save(directory, result, preview, stem):
    directory.mkdir(parents=True,exist_ok=True)
    stage = directory/f'.{stem}.json'
    stage.write_text(json.dumps(result,indent=2)+'\n')
    stage.replace(directory/f'{stem}.json')
    if preview is not None:
        stage = directory/f'.{stem}.jpg'
        cv2.imwrite(str(stage),preview)
        stage.replace(directory/f'{stem}.jpg')


def recording_frames(folder, tolerance):
    events = {'left':[],'right':[]}
    for line in (folder/'events.jsonl').read_text().splitlines():
        row = json.loads(line)
        if row.get('kind') == 'frame':
            if 'stamp' not in row:
                row['stamp'] = row['capture_ns'] / 1e9
            events[row['side']].append(row)
    for rows in events.values():
        rows.sort(key=lambda row:row['stamp'])
    times = [row['stamp'] for row in events['right']]
    with (folder/'left.mjpeg').open('rb') as l, (folder/'right.mjpeg').open('rb') as r:
        for left in events['left'][::8]:
            n = bisect_left(times,left['stamp'])
            sides = events['right'][max(0,n-1):n+1]
            if not sides:
                continue
            right = min(sides,key=lambda item:abs(item['stamp']-left['stamp']))
            skew = abs(right['stamp']-left['stamp'])
            if skew > tolerance:
                continue
            pair = []
            for row, stream in zip((left,right),(l,r)):
                stream.seek(row['offset'])
                pair.append(decode(stream.read(row['length'])))
            yield pair,left['stamp'],skew


def snapshot(args, mapper):
    results = []
    for pair,stamp,skew in recording_frames(args.recording,args.max_skew_ms/1000):
        result,preview = mapper.process(pair,stamp)
        result['pair_skew_ms'] = skew*1000
        results.append((result,preview))
    if not results:
        raise SystemExit('No synchronized stereo frames')
    detected = sorted({tag for result,_ in results for tag in result['detected_ids']})
    if args.marker_id is not None:
        relevant = [(row,preview) for row,preview in results
                    if row['tracked'] and row['pose']['marker_id'] == args.marker_id]
        if not relevant:
            save(args.output,dict(requested_marker_id=args.marker_id,detected_ids=detected,
                                  sampled_pairs=len(results),tracked=False,status='requested_marker_pose_missing'),
                 results[len(results)//2][1],'snapshot')
            raise SystemExit(f'No map pose from ID {args.marker_id}. Detected IDs: {detected}')
        results = relevant
    def rank(entry):
        row = entry[0]
        joined_marker = any(g['kind'] == 'flush_id23' for g in row['joined_gaps'])
        return row['tracked'], bool(row['lines_xy_m']), joined_marker, len(row['points_xy_m'])

    result,preview = max(results,key=rank)
    result['sampled_pairs'] = len(results)
    result['pairs_with_pose'] = sum(r['tracked'] for r,_ in results)
    result['pairs_with_white_wall'] = sum(bool(r['lines_xy_m']) for r,_ in results)
    result['pairs_bridged_behind_marker'] = sum(any(g['kind']=='flush_id23' for g in r['joined_gaps']) for r,_ in results)
    result['pairs_with_visible_upper_edge_extension'] = sum(bool(r['upper_edge_extra_samples_px']) for r,_ in results)
    save(args.output,result,preview,'snapshot')
    edges = [{'xy_m':[a,b]} for line in result['lines_xy_m'] for a,b in zip(line,line[1:])]
    if edges:
        vertices,faces = mesh(edges,panel_bottom=PANEL_BASE,panel_top=PANEL_BASE+PANEL_HEIGHT)
        with (args.output/'snapshot-mesh.txt').open('w') as f:
            print(len(vertices),len(faces),file=f)
            for vertex in vertices:
                print(*vertex,file=f)
            for face in faces:
                print(*face,file=f)
    print({k:v for k,v in result.items() if k not in ('points_xy_m','lines_xy_m')})


def live(args, mapper):
    import rclpy
    from geometry_msgs.msg import Point, PoseStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = rclpy.create_node('jetbot_live_white_walls')
    state_pub = node.create_publisher(String,'/white_walls/tracking',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pose_pub = node.create_publisher(PoseStamped,'/white_walls/aruco_pose',5)
    map_pub = node.create_publisher(MarkerArray,'/white_walls/geometry',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    frames = {side:deque(maxlen=6) for side in ('left','right')}
    last_pair,last_seen = -float('inf'),-float('inf')
    expired = False
    last_preview = -float('inf')

    def xyz(x,y,z):
        return Point(x=float(x),y=float(y),z=float(z))

    def publish(result):
        stamp = node.get_clock().now().to_msg()
        state_pub.publish(String(data=json.dumps({k:v for k,v in result.items()
                                                    if k not in ('points_xy_m','lines_xy_m','upper_edge_extra_samples_px')})))
        def item(i, kind, color, scale=1.):
            msg = Marker()
            msg.header.frame_id = result['frame_id']
            msg.header.stamp = stamp
            msg.ns = 'live_white_walls'
            msg.id,msg.type,msg.action = i,kind,Marker.ADD
            msg.scale.x = msg.scale.y = msg.scale.z = scale
            msg.pose.orientation.w = 1.
            msg.color.r,msg.color.g,msg.color.b,msg.color.a = color
            msg.lifetime.nanosec = 800_000_000
            return msg
        clear = item(0,Marker.POINTS,(0.,0.,0.,0.))
        clear.action = Marker.DELETEALL
        points = item(1,Marker.POINTS,(.1,.83,.85,1.),.012)
        lines = item(2,Marker.LINE_LIST,(1.,.64,.16,1.),.015)
        face = item(3,Marker.TRIANGLE_LIST,(.89,.89,.88,.55))
        points.points = [xyz(x,y,PANEL_BASE) for x,y in result.get('points_xy_m',[])]
        for curve in result.get('lines_xy_m',[]):
            for (x,y),(u,v) in zip(curve,curve[1:]):
                a,b,c,d = xyz(x,y,PANEL_BASE),xyz(u,v,PANEL_BASE),xyz(u,v,PANEL_BASE+PANEL_HEIGHT),xyz(x,y,PANEL_BASE+PANEL_HEIGHT)
                lines.points.extend((a,b,c,d))
                face.points.extend((a,b,c,a,c,d))
        markers = [clear,points,lines,face]
        if result['tracked']:
            pose = result['pose']
            msg = PoseStamped()
            msg.header = clear.header
            msg.pose.position = xyz(*pose['xy_m'],0.)
            msg.pose.orientation.z = math.sin(pose['yaw_rad']/2)
            msg.pose.orientation.w = math.cos(pose['yaw_rad']/2)
            pose_pub.publish(msg)
            arrow = item(4,Marker.ARROW,(.1,.95,.22,1.))
            arrow.pose = msg.pose
            arrow.scale.x,arrow.scale.y,arrow.scale.z = .22,.06,.06
            markers.append(arrow)
        map_pub.publish(MarkerArray(markers=markers))

    def intake(message, side):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        frames[side].append((stamp,bytes(message.data),monotonic()))

    def tick():
        nonlocal last_pair,last_seen,expired,last_preview
        now = monotonic()
        while frames['left'] and frames['right']:
            l = frames['left'].popleft()
            choices = [(abs(row[0]-l[0]),index,row) for index,row in enumerate(frames['right'])]
            delta,index,r = min(choices)
            if delta > args.max_skew_ms/1000 or now-min(l[2],r[2])>POSE_TIMEOUT or l[0]<=last_pair:
                continue
            del frames['right'][index]
            last_pair = l[0]
            try:
                result,preview = mapper.process([decode(l[1]),decode(r[1])],l[0])
            except (ValueError,cv2.error) as error:
                node.get_logger().warn(str(error))
                continue
            result['pair_skew_ms'] = delta*1000
            last_seen,expired = now,False
            publish(result)
            if now-last_preview >= 1.:
                save(args.output,result,preview,'latest')
                last_preview = now
            break
        if now-last_seen > POSE_TIMEOUT and not expired:
            result = dict(stamp_s=None,frame_id='map',tracked=False,pose=None,status='stereo_frame_timeout')
            publish(result)
            save(args.output,result,None,'latest')
            expired = True

    for side in frames:
        node.create_subscription(CompressedImage,f'/stereo/{side}/image_raw/compressed',
                                 lambda msg,s=side:intake(msg,s),qos_profile_sensor_data)
    node.create_timer(.25,tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration',type=Path,required=True)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--recording',type=Path,help='Process a recorded compressed ROS stream once')
    parser.add_argument('--max-skew-ms',type=float,default=FRAME_TOLERANCE*1000)
    parser.add_argument('--marker-id',type=int,help='Select snapshots localized by this registered marker')
    args = parser.parse_args()
    if args.marker_id is not None and (not args.recording or not 0 <= args.marker_id < 50):
        parser.error('marker-id requires a recording and an ID between 0 and 49')
    if not 0 < args.max_skew_ms <= 35:
        parser.error('maximum skew must be in (0, 35] ms')
    cv2.setNumThreads(2)
    mapper = Mapper(args.calibration,args.map)
    (snapshot if args.recording else live)(args,mapper)


if __name__ == '__main__':
    main()
