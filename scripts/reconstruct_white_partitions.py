#!/usr/bin/env python3
"""Build a sparse mesh of observed white partitions from aligned stereo frames."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

PANEL_BOTTOM = 0.003
PANEL_TOP = 0.203
PANEL_THICKNESS = 0.003


def preprocess(image):
    denoised = cv2.medianBlur(image, 3)
    lab = cv2.cvtColor(denoised, cv2.COLOR_BGR2LAB)
    luma = lab[:, :, 0]
    contrast = cv2.createCLAHE(2.0, (8, 8)).apply(luma)
    # Strong edges in any Lab channel are retained, including floor color changes.
    gradient = np.max(np.abs(cv2.Sobel(lab, cv2.CV_32F, 0, 1, ksize=3)), axis=2)
    return lab, contrast, gradient


def line_candidates(luma, contrast, detector):
    candidates = []
    for image in (luma, contrast):
        lines = detector.detect(image)[0]
        if lines is None:
            continue
        for x0, y0, x1, y1 in lines[:, 0]:
            if x0 > x1:
                x0, y0, x1, y1 = x1, y1, x0, y0
            if x1 - x0 < 32 or abs(y1 - y0) > (x1 - x0) * .78:
                continue
            slope = (y1 - y0) / (x1 - x0)
            candidates.append((slope, y0 - slope * x0, x0, x1))
    return sorted(candidates, key=lambda edge: edge[3] - edge[2], reverse=True)[:65]


def edge_strength(gradient, x, y, radius=2):
    cols = np.rint(x).astype(int)
    rows = np.rint(y).astype(int)
    return np.max(gradient[rows[:, None] + np.arange(-radius, radius + 1), cols[:, None]], axis=1)


def extract(left, right, camera, mount, detector, marker_detector, *, dense=False,
            panel_height=PANEL_TOP - PANEL_BOTTOM, panel_bottom=PANEL_BOTTOM, right_edge_radius=2):
    fx, fy, cx, cy, baseline = camera
    height, nominal_pitch = mount
    lab, contrast, left_grad = preprocess(left)
    _, _, right_grad = preprocess(right)
    lines = line_candidates(lab[:, :, 0], contrast, detector)
    markers = marker_detector(left)
    marker_mask = np.zeros(lab.shape[:2], dtype=np.uint8)
    for marker in markers:
        corners = marker.reshape(4, 2)
        center = corners.mean(axis=0)
        padded = np.rint(center + 1.27 * (corners - center)).astype(np.int32)
        cv2.fillConvexPoly(marker_mask, padded, 1)
    h, w = lab.shape[:2]
    accepted = []
    for tm, tb, tl, tr in lines:
        top_plane = np.array([-tm * fx, fy, cy - tm * cx - tb])
        for bm, bb, bl, br in lines:
            lo, hi = max(tl, bl), min(tr, br)
            if hi - lo < 33:
                continue
            x = np.arange(np.ceil(lo) + 1, np.floor(hi), 4.)
            bottom = bm * x + bb
            top = tm * x + tb
            if np.any(bottom - top < 44) or np.any(bottom - top > 420):
                continue
            if np.any(top < 4) or np.any(bottom >= h - 4):
                continue
            fraction = (height - panel_bottom) / panel_height
            horizon = bottom + fraction * (top - bottom)
            pitch = np.arctan2(cy - np.median(horizon), fy)
            if abs(pitch - nominal_pitch) > np.radians(4.5):
                continue
            sine, cosine = np.sin(pitch), np.cos(pitch)
            up = np.array([0., -cosine, -sine])
            ray = np.column_stack(((x - cx) / fx, (bottom - cy) / fy, np.ones(len(x))))
            depth = -panel_height * (top_plane @ up) / (bottom - top)
            foot = ray * depth[:, None]
            cap = foot + panel_height * up
            lower_z = height + foot @ up
            # Height from image separation implies a depth. The floor-height and
            # second-image checks independently reject false edge bands.
            plausible = ((depth >= .30) & (depth <= 2.7)
                         & (abs(lower_z - panel_bottom) <= .024))
            if np.count_nonzero(plausible) < 8:
                continue
            rx0 = fx * (foot[:, 0] - baseline) / depth + cx
            rx1 = fx * (cap[:, 0] - baseline) / cap[:, 2] + cx
            ry1 = fy * cap[:, 1] / cap[:, 2] + cy
            plausible &= ((rx0 > 3) & (rx1 > 3) & (rx0 < w - 4) & (rx1 < w - 4)
                          & (ry1 > right_edge_radius + 1) & (ry1 < h - right_edge_radius - 2)
                          & (bottom > right_edge_radius + 1) & (bottom < h - right_edge_radius - 2))
            idx = np.flatnonzero(plausible)
            if len(idx) < 8:
                continue
            l0 = edge_strength(left_grad, x[idx], bottom[idx])
            l1 = edge_strength(left_grad, x[idx], top[idx])
            r0 = edge_strength(right_grad, rx0[idx], bottom[idx], right_edge_radius)
            r1 = edge_strength(right_grad, rx1[idx], ry1[idx], right_edge_radius)
            # Interior between the edges should mostly be light and neutral.
            rows = (top[idx][None, :] + np.array([.06, .25, .50, .75, .94])[:, None]
                    * (bottom - top)[idx][None, :])
            interior = lab[np.rint(rows).astype(int), x[idx].astype(int)].astype(float)
            light = np.median(interior[:, :, 0], axis=0)
            chroma = np.linalg.norm(interior[:, :, 1:] - 128, axis=2)
            neutral = np.median(chroma, axis=0) < 20
            max_luma_gap = np.max(abs(interior[:, :, 0] - light[None, :]), axis=0)
            clear = ~np.any(marker_mask[np.rint(rows).astype(int), x[idx].astype(int)], axis=0)
            hits = ((np.minimum.reduce([l0, l1, r0, r1]) >= 9) & neutral
                    & (light >= 82) & (max_luma_gap <= 24) & clear)
            for run in np.split(idx[hits], np.flatnonzero(np.diff(idx[hits]) > 1) + 1):
                if len(run) < 8:
                    continue
                samples = run if dense else run[[0, -1]]
                optical = foot[samples]
                xy = np.column_stack((optical[:, 2] * cosine - optical[:, 1] * sine,
                                      -optical[:, 0]))
                if np.linalg.norm(xy[-1] - xy[0]) < .075:
                    continue
                pixels = np.column_stack((x[samples], top[samples], bottom[samples]))
                accepted.append((pixels, xy, len(run)))
    # Keep the widest validated band per column; repeated LSD edges give duplicates.
    selected = []
    occupied = np.zeros(w, bool)
    for pixels, xy, count in sorted(accepted, key=lambda entry: -entry[2]):
        span = slice(int(pixels[0, 0]), int(pixels[-1, 0]) + 1)
        if np.any(occupied[span]):
            continue
        occupied[span] = True
        selected.append((pixels, xy))
    return selected


def fuse(observations, min_frames=3):
    # This room uses axis-aligned walls. Leave large gaps and unsupported runs open.
    segments = []
    for axis in (0, 1):
        perpendicular = 1 - axis
        entries = []
        for frame, endpoints in observations:
            delta = np.abs(endpoints[1] - endpoints[0])
            if delta[axis] < .10 or delta[perpendicular] > .32 * delta[axis]:
                continue
            lo, hi = sorted(endpoints[:, axis])
            entries.append((frame, lo, hi, np.mean(endpoints[:, perpendicular])))
        remaining = entries
        while remaining:
            best = max(remaining, key=lambda seed: sum(abs(row[3] - seed[3]) <= .055 for row in remaining))
            group = [row for row in remaining if abs(row[3] - best[3]) <= .055]
            remaining = [row for row in remaining if abs(row[3] - best[3]) > .055]
            if len({row[0] for row in group}) < min_frames:
                continue
            bounds = sorted({bound for row in group for bound in row[1:3]})
            intervals = []
            for lo, hi in zip(bounds, bounds[1:]):
                mid = (lo + hi) / 2
                support = {i for i, a, b, _ in group if a <= mid <= b}
                if len(support) < min_frames:
                    continue
                if intervals and np.isclose(intervals[-1][1], lo):
                    intervals[-1][1] = hi
                    intervals[-1][2].update(support)
                else:
                    intervals.append([lo, hi, support])
            for lo, hi, support in intervals:
                if hi - lo < .15:
                    continue
                endpoints = np.zeros((2, 2))
                endpoints[:, axis] = [lo, hi]
                endpoints[:, perpendicular] = np.median(
                    [row[3] for row in group if row[0] in support])
                segments.append(dict(xy_m=endpoints.tolist(), frames=len(support), length_m=hi-lo))
    return segments


def mesh(segments, *, panel_bottom=PANEL_BOTTOM, panel_top=PANEL_TOP, thickness=PANEL_THICKNESS):
    vertices, faces = [], []
    for segment in segments:
        start, end = np.array(segment['xy_m'])
        # Each panel is a closed rectangular solid with measured thickness.
        along = (end - start) / np.linalg.norm(end - start)
        across = np.array([-along[1], along[0]]) * thickness / 2
        ring = [start - across, end - across, end + across, start + across]
        offset = len(vertices)
        vertices.extend((x, y, z) for z in (panel_bottom, panel_top) for x, y in ring)
        for quad in [(3, 2, 1, 0), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5),
                     (2, 3, 7, 6), (3, 0, 4, 7)]:
            a, b, c, d = (offset + i for i in quad)
            faces.extend(((a, b, c), (a, c, d)))
    return vertices, faces


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('frames', type=Path, help='rectified left/right JPEG folders and camera.txt')
    parser.add_argument('poses', type=Path, help='index timestamp x y yaw section TSV')
    parser.add_argument('output', type=Path)
    parser.add_argument('--reference', type=Path, required=True, help='Reference occupancy YAML')
    parser.add_argument('--height', type=float, default=.099)
    parser.add_argument('--pitch', type=float, default=3.5)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    camera = tuple(map(float, (args.frames / 'camera.txt').read_text().split()))
    reference = yaml.safe_load(args.reference.read_text())
    grid = cv2.imread(str(args.reference.parent / reference['image']), cv2.IMREAD_GRAYSCALE)
    if grid is None:
        raise ValueError('Reference occupancy image could not be opened')
    occupied = grid < 50
    horizontal = np.zeros_like(occupied)
    vertical = np.zeros_like(occupied)
    horizontal[1:, :] = occupied[1:, :] != occupied[:-1, :]
    vertical[:, 1:] = occupied[:, 1:] != occupied[:, :-1]
    distance_grids = [cv2.distanceTransform(np.uint8(~edges), cv2.DIST_L2, 5)
                      for edges in (horizontal, vertical)]
    origin = np.array(reference['origin'][:2])
    resolution = reference['resolution']

    def near_reference(points):
        samples = np.linspace(*points, 12)
        cols, floor_rows = np.floor((samples - origin) / resolution).astype(int).T
        rows = grid.shape[0] - 1 - floor_rows
        if np.any((cols < 0) | (rows < 0) | (cols >= grid.shape[1]) | (rows >= grid.shape[0])):
            return False
        axis = np.argmax(abs(points[1] - points[0]))
        return np.max(distance_grids[axis][rows, cols]) * resolution <= .125

    detector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        aruco = cv2.aruco.ArucoDetector(dictionary)
        marker_detector = lambda image: aruco.detectMarkers(image)[0]
    else:
        marker_detector = lambda image: cv2.aruco.detectMarkers(image, dictionary)[0]
    cv2.setNumThreads(2)
    observations, previews = [], {}
    for index, _, x, y, heading, section in np.loadtxt(args.poses, ndmin=2):
        label = f'{int(index):05d}.jpg'
        left, right = (cv2.imread(str(args.frames / side / label)) for side in ('left', 'right'))
        if left is None or right is None:
            raise ValueError(f'Missing stereo frame {label}')
        matches = extract(left, right, camera, (args.height, np.radians(args.pitch)), detector, marker_detector)
        cs, sn = np.cos(heading), np.sin(heading)
        rotation = np.array([[cs, -sn], [sn, cs]])
        accepted = False
        for pixels, xy in matches:
            endpoints = xy @ rotation.T + [x, y]
            if not near_reference(endpoints):
                continue
            observations.append((int(index), endpoints))
            accepted = True
            for edge, color in [(1, (0, 190, 255)), (2, (65, 220, 0))]:
                pts = np.rint(pixels[:, [0, edge]]).astype(int)
                cv2.line(left, tuple(pts[0]), tuple(pts[1]), color, 2)
                for point in pts:
                    cv2.circle(left, tuple(point), 4, color, -1)
        if accepted and section not in previews:
            cv2.putText(left, f'Frame {int(index)}', (8, 25), 0, .7, (0, 255, 255), 2)
            previews[section] = cv2.resize(left, (320, 240))
    segments = fuse(observations)
    vertices, faces = mesh(segments)
    report = dict(coordinate_frame='map2_room', panel_bottom_m=PANEL_BOTTOM,
                  panel_top_m=PANEL_TOP, thickness_m=PANEL_THICKNESS,
                  max_distance_from_reference_m=.125,
                  matched_frames=len({i for i, _ in observations}),
                  band_observations=len(observations), segments=segments)
    (args.output / 'white-partitions.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    if previews:
        images = list(previews.values())
        if len(images) % 2:
            images.append(np.zeros_like(images[0]))
        montage = np.vstack([np.hstack(images[i:i+2]) for i in range(0, len(images), 2)])
        cv2.imwrite(str(args.output / 'white-partitions-edges.jpg'), montage)
    if not faces:
        raise SystemExit('No wall sections met the evidence threshold')
    with (args.output / 'white-partitions-mesh.txt').open('w') as output:
        print(len(vertices), len(faces), file=output)
        for x, y, z in vertices:
            print(x, y, z, 235, 235, 235, file=output)
        for face in faces:
            print(*face, file=output)


if __name__ == '__main__':
    main()
