#!/usr/bin/env python3
"""Compare synchronized ArUco geometry with a stationary tape-measured wall reference."""
import argparse
from bisect import bisect_left
import csv
import json
from time import perf_counter
from pathlib import Path

import cv2
import numpy as np
import yaml


def observations(folder, marker_id):
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = (aruco.DetectorParameters() if hasattr(aruco, 'ArucoDetector')
              else aruco.DetectorParameters_create())
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    detect = (aruco.ArucoDetector(dictionary, params).detectMarkers
              if hasattr(aruco, 'ArucoDetector')
              else lambda img: aruco.detectMarkers(img, dictionary, parameters=params))
    sides = []
    for sensor in (0, 1):
        with (folder / f'camera-{sensor}-timestamps.csv').open() as stream:
            times = {int(row['frame_index']): int(row['clock_time_ns'])
                     for row in csv.DictReader(stream) if int(row['clock_time_ns']) > 0}
        video = cv2.VideoCapture(str(folder / f'camera-{sensor}.avi'))
        samples = []
        index = 0
        try:
            while True:
                ok, image = video.read()
                if not ok:
                    break
                corners, ids, _ = detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
                if ids is not None and list(ids.ravel()).count(marker_id) == 1 and index in times:
                    points = corners[list(ids.ravel()).index(marker_id)].reshape(4, 2)
                    if cv2.contourArea(points) >= 350:
                        samples.append((times[index], points, index))
                index += 1
        finally:
            video.release()
        sides.append(samples)
    right_times = [time for time, _, _ in sides[1]]
    pairs = []
    used = set()
    for left in sides[0]:
        pos = bisect_left(right_times, left[0])
        options = sides[1][max(0, pos - 1):pos + 1]
        if options:
            right = min(options, key=lambda row: abs(left[0] - row[0]))
            if abs(left[0] - right[0]) <= 35_000_000 and right[2] not in used:
                pairs.append((left, right))
                used.add(right[2])
    return pairs


def calibration(folder):
    cameras = []
    for side in ('left', 'right'):
        data = yaml.safe_load((folder / f'{side}.yaml').read_text())
        def matrix(key, shape):
            return np.asarray(data[key]['data'], dtype=float).reshape(shape)
        cameras.append((matrix('camera_matrix', (3, 3)),
                        np.asarray(data['distortion_coefficients']['data']),
                        matrix('rectification_matrix', (3, 3)),
                        matrix('projection_matrix', (3, 4))))
    return cameras


def measure(pairs, cameras, ref):
    half = ref['marker_nominal_side_m'] / 2
    square = half * np.array([[-1., 1., 0.], [1., 1., 0.], [1., -1., 0.], [-1., -1., 0.]])
    p1, p2 = (cam[3] for cam in cameras)
    stats = []
    centers = []
    for left, right in pairs:
        pixels = []
        for (_, points, _), (k, d, r, p) in zip((left, right), cameras):
            pixels.append(cv2.undistortPoints(points.astype(float), k, d, R=r, P=p).reshape(4, 2))
        l, r = pixels
        homogeneous = cv2.triangulatePoints(p1, p2, l.T, r.T)
        points = (homogeneous[:3] / homogeneous[3]).T
        if np.any(points[:, 2] <= 0):
            continue
        center = points.mean(axis=0)
        centers.append(center)
        left_k, left_d = cameras[0][:2]
        ok, rotation, translation = cv2.solvePnP(square, left[1], left_k, left_d,
                                                flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        predicted = cv2.projectPoints(square, rotation, translation, left_k, left_d)[0].reshape(4, 2)
        stats.append([abs(left[0] - right[0]) / 1e6,
                      np.median(abs(l[:, 1] - r[:, 1])),
                      np.max(abs(l[:, 1] - r[:, 1])),
                      np.linalg.norm(center), center[2], np.linalg.norm(translation),
                      np.sqrt(np.mean(np.sum((left[1] - predicted)**2, axis=1)))])
    if not stats:
        return {'usable_pairs': 0}
    skew, row_error, worst, stereo_range, optical_depth, pnp_range, reprojection = np.median(stats, axis=0)
    baseline = abs(p2[0, 3] / p2[0, 0] - p1[0, 3] / p1[0, 0])
    left_corners = np.array([pair[0][1] for pair in pairs])
    movement = np.linalg.norm(left_corners - np.median(left_corners, axis=0), axis=2)
    return dict(usable_pairs=len(stats),
                left_corner_motion_p95_px=float(np.percentile(movement, 95)), fx_rect_px=float(p1[0, 0]), calibration_baseline_m=float(baseline),
                median_capture_skew_ms=skew, median_row_difference_px=row_error,
                median_worst_corner_row_difference_px=worst,
                rectification_check_passed=bool(row_error < 1 and worst < 1),
                stereo_center_rectified_m=np.median(centers, axis=0).tolist(),
                stereo_range_m=stereo_range, stereo_optical_depth_m=optical_depth,
                stereo_minus_tape_m=stereo_range - ref['left_lens_to_center_m'],
                left_pnp_range_m=pnp_range, left_pnp_reprojection_rms_px=reprojection,
                depth_sensitivity_m_at_half_px=float(optical_depth**2 * .5 / (p1[0, 0] * baseline)))


def inspect_wall(folder, pair, cameras, ref, preview):
    frames = []
    for sensor, sample in enumerate(pair):
        video = cv2.VideoCapture(str(folder / f'camera-{sensor}.avi'))
        try:
            video.set(cv2.CAP_PROP_POS_FRAMES, sample[2])
            ok, image = video.read()
            if not ok:
                raise ValueError('Could not reopen the selected stereo pair')
            frames.append(image)
        finally:
            video.release()
    half = ref['marker_nominal_side_m'] / 2
    square = half * np.array([[-1., 1., 0.], [1., 1., 0.], [1., -1., 0.], [-1., -1., 0.]])
    graph = []
    rectified = []
    wall_polygon = None
    marker_polygon = None
    plane_normal = None
    plane_offset = None
    for image, sample, (k, d, r, p) in zip(frames, pair, cameras):
        ok, rotation, translation = cv2.solvePnP(square, sample[1], k, d,
                                                flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            raise ValueError('Could not estimate wall reference pose for preview')
        left_edge = -ref['center_from_panel_left_edge_m']
        bottom = -ref['center_above_panel_bottom_m']
        top = ref['derived_center_below_panel_top_m']
        # Only draw the wall span from the surveyed left corner to the marker center.
        wall = np.array([[left_edge, top, 0], [0, top, 0], [0, bottom, 0], [left_edge, bottom, 0]])
        marked = image.copy()
        projected = cv2.projectPoints(wall, rotation, translation, k, d)[0].reshape(4, 2)
        origin = cv2.projectPoints(np.zeros((1, 3)), rotation, translation, k, d)[0].reshape(2)
        corners = np.rint(projected).astype(int)
        cv2.line(marked, tuple(corners[0]), tuple(corners[1]), (0, 190, 255), 2)
        cv2.line(marked, tuple(corners[3]), tuple(corners[2]), (65, 220, 0), 2)
        cv2.line(marked, tuple(corners[0]), tuple(corners[3]), (255, 190, 0), 2)
        cv2.circle(marked, tuple(np.rint(origin).astype(int)), 5, (0, 0, 255), 2)
        graph.append(marked)
        grid = cv2.initUndistortRectifyMap(k, d, r, p, image.shape[1::-1], cv2.CV_16SC2)
        rectified.append(cv2.cvtColor(cv2.remap(image, *grid, cv2.INTER_LINEAR), cv2.COLOR_BGR2GRAY))
        if wall_polygon is None:
            wall_polygon = cv2.undistortPoints(projected, k, d, R=r, P=p).reshape(4, 2)
            marker_polygon = cv2.undistortPoints(sample[1], k, d, R=r, P=p).reshape(4, 2)
            plane_normal = (r @ cv2.Rodrigues(rotation)[0])[:, 2]
            plane_offset = float(plane_normal @ (r @ translation).ravel())
    if not cv2.imwrite(str(preview), np.hstack(graph)):
        raise OSError(f'Could not write {preview}')
    # This benchmark uses the laptop CPU. Disparities are screened by the
    # matcher; usable pixel fractions still depend on stereo calibration.
    matcher = cv2.StereoSGBM_create(minDisparity=0, numDisparities=160, blockSize=5,
                                   P1=200, P2=800, disp12MaxDiff=1, uniquenessRatio=12,
                                   speckleWindowSize=50, speckleRange=2,
                                   mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    matcher.compute(*rectified)
    times = []
    for _ in range(10):
        start = perf_counter()
        disparity = matcher.compute(*rectified) / 16.
        times.append(1000 * (perf_counter() - start))

    def polygon_mask(points):
        mask = np.zeros(disparity.shape, dtype=np.uint8)
        cv2.fillConvexPoly(mask, np.rint(points).astype(np.int32), 1)
        return mask

    marker_mask = polygon_mask(marker_polygon)
    wall_mask = polygon_mask(wall_polygon)
    wall_mask[cv2.dilate(marker_mask, np.ones((35, 35), np.uint8)) > 0] = 0
    wall_mask = cv2.erode(wall_mask, np.ones((21, 21), np.uint8))
    # The matcher cannot estimate the first numDisparities columns.
    wall_mask[:, :160] = 0
    marker_mask[:, :160] = 0
    valid = disparity > 0

    def fraction(mask):
        pixels = int(np.count_nonzero(mask))
        return (float(np.count_nonzero(valid & (mask > 0)) / pixels) if pixels else None, pixels)

    wall_fraction, wall_pixels = fraction(wall_mask)
    marker_fraction, marker_pixels = fraction(marker_mask)
    fx = cameras[0][3][0, 0]
    k_rect = cameras[0][3][:, :3]
    y, x = np.indices(disparity.shape)
    rays = np.stack(((x - k_rect[0, 2]) / fx,
                     (y - k_rect[1, 2]) / k_rect[1, 1], np.ones_like(x)), axis=-1)
    baseline = -cameras[1][3][0, 3] / cameras[1][3][0, 0]
    predicted_disparity = fx * baseline * (rays @ plane_normal) / plane_offset
    wall_error = abs(disparity - predicted_disparity)[valid & (wall_mask > 0)]
    return dict(device='laptop CPU', image_size_px=list(disparity.shape[::-1]),
                threads=2, disparities=160, stationary_frame_indices=[sample[2] for sample in pair],
                median_pair_ms=float(np.median(times)), wall_return_fraction=wall_fraction,
                wall_pixel_count=wall_pixels,
                wall_disparity_error_vs_marker_plane_median_px=float(np.median(wall_error)) if wall_error.size else None,
                marker_return_fraction=marker_fraction,
                marker_pixel_count=marker_pixels)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video_folder', type=Path)
    parser.add_argument('reference_json', type=Path)
    parser.add_argument('--calibration', type=Path, required=True, action='append')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preview', type=Path, help='Draw the measured wall offsets using the best rectification candidate')
    args = parser.parse_args()
    ref = json.loads(args.reference_json.read_text())
    cv2.setNumThreads(2)
    pairs = observations(args.video_folder, ref['marker_id'])
    result = {'video': str(args.video_folder), 'reference': ref, 'detected_pairs': len(pairs),
              'calibrations': {str(folder): measure(pairs, calibration(folder), ref)
                               for folder in args.calibration}}
    compared = {folder: result['calibrations'][str(folder)] for folder in args.calibration}
    usable = [folder for folder, stats in compared.items() if stats['usable_pairs']]
    if args.preview and usable:
        best = min(usable, key=lambda folder: compared[folder]['median_row_difference_px'])
        result['sgbm_diagnostic'] = inspect_wall(args.video_folder, pairs[len(pairs) // 2],
                                                 calibration(best), ref, args.preview)
        result['sgbm_diagnostic']['calibration'] = str(best)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if not usable:
        raise SystemExit('No usable synchronized marker pairs')


if __name__ == '__main__':
    main()
