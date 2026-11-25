"""Camera-relative clearance and marker observations for the recorded JetBot route."""
import math

import cv2
import numpy as np

from jetbot_stereo.boundary_walls import model_partitions
from jetbot_stereo.calibration import load_calibration, validate_pair


class RouteVision:
    def __init__(self, calibration, pitch_deg=3.5, guard_scale=1.):
        cameras = [load_calibration(calibration / f'{side}.yaml', 640, 480)
                   for side in ('left', 'right')]
        validate_pair(*cameras)
        self.pitch_deg = pitch_deg
        self.height = .099
        self.maps = []
        for camera in cameras:
            k = np.asarray(camera['camera_matrix']['data']).reshape(3, 3)
            d = np.asarray(camera['distortion_coefficients']['data'])
            r = np.asarray(camera['rectification_matrix']['data']).reshape(3, 3)
            p = np.asarray(camera['projection_matrix']['data']).reshape(3, 4)
            self.maps.append(cv2.initUndistortRectifyMap(k, d, r, p, (640, 480), cv2.CV_32FC1))
        self.p = p = np.asarray(cameras[0]['projection_matrix']['data']).reshape(3, 4)
        self.r = np.asarray(cameras[0]['rectification_matrix']['data']).reshape(3, 3)
        right = np.asarray(cameras[1]['projection_matrix']['data']).reshape(3, 4)
        self.baseline = abs(right[0, 3] / right[0, 0])
        c, s = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
        y, x = np.indices((480, 640), dtype=np.float32)
        rays = np.stack(((x - p[0, 2]) / p[0, 0], (y - p[1, 2]) / p[1, 1], np.ones_like(x)), axis=-1)
        level = rays @ self.r @ np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        self.level_rays = level.astype(np.float32)
        self.valid_maps = [(u >= 1) & (u < 638) & (v >= 1) & (v < 478)
                           for u, v in self.maps]
        self.guard_scale = guard_scale
        self.guard_size = (int(640 * guard_scale), int(480 * guard_scale))
        self.guard_projection = p.copy()
        self.guard_projection[:2] *= guard_scale
        self.guard_projection[:2, 2] += (guard_scale - 1) / 2
        self.guard_rays = cv2.resize(self.level_rays, self.guard_size, interpolation=cv2.INTER_LINEAR)
        self.guard_masks = [cv2.resize(mask.astype(np.uint8), self.guard_size,
                                      interpolation=cv2.INTER_NEAREST).astype(bool)
                            for mask in self.valid_maps]
        self.rows, self.columns = np.indices(self.guard_size[::-1])
        self.disparity_pad = int(math.ceil(320 * guard_scale / 16)) * 16
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=self.disparity_pad, blockSize=5, P1=200, P2=800,
            disp12MaxDiff=1, uniquenessRatio=12,
            speckleWindowSize=max(12, int(70 * guard_scale ** 2)), speckleRange=2,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
        )
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        if hasattr(aruco, 'ArucoDetector'):
            parameters = aruco.DetectorParameters()
            parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self.detect = aruco.ArucoDetector(dictionary, parameters).detectMarkers
        else:
            parameters = aruco.DetectorParameters_create()
            parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
            self.detect = lambda image: aruco.detectMarkers(image, dictionary, parameters=parameters)

    def rectify(self, frames):
        return [cv2.remap(frame, *maps, cv2.INTER_LINEAR)
                for frame, maps in zip(frames, self.maps, strict=True)]

    def markers(self, gray):
        detections = []
        for image in gray:
            corners, ids, _ = self.detect(image)
            detections.append({int(i): c.reshape(4, 2) for i, c in zip(
                ids.ravel() if ids is not None else (), corners
            )})
        observations = {}
        for identifier in detections[0].keys() & detections[1].keys():
            left, right = (results[identifier] for results in detections)
            disparity = left[:, 0] - right[:, 0]
            if disparity.min() < 12 or np.max(np.abs(left[:, 1] - right[:, 1])) > 4:
                continue
            x, z = (self.level_rays[
                np.round(left[:, 1]).astype(int).clip(0, 479),
                np.round(left[:, 0]).astype(int).clip(0, 639)
            ] * (self.baseline * self.p[0, 0] / disparity[:, None])).mean(axis=0)[[0, 2]]
            observations[identifier] = (float(math.hypot(x, z)), math.degrees(math.atan2(x, z)))
        return observations

    def disparity(self, left, right):
        # Padding removes the left-edge search restriction of SGBM. Remap validity
        # and the reciprocal match still exclude rays that fall outside either eye.
        pad = self.disparity_pad
        a, b = (cv2.copyMakeBorder(image, 0, 0, pad, 0, cv2.BORDER_CONSTANT)
                for image in (left, right))
        return self.matcher.compute(a, b)[:, pad:].astype(np.float32) / 16

    def inspect(self, frames):
        checks, markers = self.inspect_sectors(frames, (0,))
        return checks[0], markers

    def inspect_sectors(self, frames, headings):
        left, right = self.rectify(frames)
        gray = [cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) for frame in (left, right)]
        markers = self.markers(gray)
        if self.guard_scale != 1:
            gray = [cv2.resize(image, self.guard_size, interpolation=cv2.INTER_AREA) for image in gray]
            left = cv2.resize(left, self.guard_size, interpolation=cv2.INTER_AREA)
        disparity = self.disparity(*gray)
        with np.errstate(divide='ignore', invalid='ignore'):
            x, down, z = (self.guard_rays *
                          (self.baseline * self.guard_projection[0, 0] / disparity[..., None])).transpose(2, 0, 1)
        reverse = self.disparity(gray[1][:, ::-1], gray[0][:, ::-1])[:, ::-1]
        projected_x = np.rint(self.columns - disparity).astype(int)
        right_x = projected_x.clip(0, self.guard_size[0] - 1)
        valid = ((disparity >= 16 * self.guard_scale) & (z > .27)
                 & (projected_x >= 0) & self.guard_masks[0] & self.guard_masks[1][self.rows, right_x]
                 & (np.abs(disparity - reverse[self.rows, right_x]) <= max(.75, 1.5 * self.guard_scale)))
        # The left optical center sits roughly half the stereo baseline left of the rig center.
        body_x = x - self.baseline / 2
        # Check the textureless partitions independently of stereo feature tracking.
        walls = model_partitions(
            left, self.guard_projection, self.r, self.height, self.pitch_deg, partition_height_m=.203,
            partition_pitch_tolerance_deg=3., min_panel_luma=100.,
        )['walls']
        panels = [np.linspace(*np.asarray(w['floor_endpoints_level']), 65)[:, [0, 2]]
                  for w in walls]
        checks = {}
        for degrees_right in headings:
            angle = math.radians(degrees_right)
            c, s = math.cos(angle), math.sin(angle)
            with np.errstate(invalid='ignore'):
                lateral = c * body_x - s * z
                forward = s * body_x + c * z
            checks[degrees_right] = self.check_lane(lateral, forward, down, valid, panels, c, s)
        return checks, markers

    def check_lane(self, lateral, forward, down, valid, panels, cosine, sine):
        near = (valid & (forward > 0.) & (forward < .45) & (abs(lateral) < .16)
                & (down < self.height - .023) & (down > -.35))
        obstacle = cv2.morphologyEx(near.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        components, _, stats, _ = cv2.connectedComponentsWithStats(obstacle)
        if components > 1 and max(stats[1:, cv2.CC_STAT_AREA]) >= max(9, 110 * self.guard_scale ** 2):
            return 'front obstacle within 45 cm'
        floor = valid & (forward >= .46) & (forward <= .75) & (abs(down - self.height) <= .021)
        lanes = ((-.15, -.05), (-.05, .05), (.05, .15))
        supported = [np.count_nonzero(floor & (lateral >= lo) & (lateral < hi))
                     >= max(8, 45 * self.guard_scale ** 2) for lo, hi in lanes]
        for px, pz in (points.T for points in panels):
            body_x = px - self.baseline / 2
            side = cosine * body_x - sine * pz
            ahead = sine * body_x + cosine * pz
            if np.any((abs(side) < .16) & (ahead > 0.) & (ahead <= .45)):
                return 'front partition within 45 cm'
            for i, (lo, hi) in enumerate(lanes):
                supported[i] |= np.count_nonzero((side >= lo) & (side < hi) & (ahead > .48)) >= 3
        return None if all(supported) else 'forward clearance unknown'

    def route_alignment(self, reference, current_left):
        """Estimate current offset from one recorded stereo waypoint, when observable."""
        left, right = self.rectify(reference)
        current = cv2.remap(current_left, *self.maps[0], cv2.INTER_LINEAR)
        orb = cv2.ORB_create(nfeatures=850, fastThreshold=10)
        keypoints, descriptors = zip(*(orb.detectAndCompute(image, None) for image in (left, right, current)))
        if any(d is None or len(d) < 2 for d in descriptors):
            return None
        matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        matches = []
        for target in (1, 2):
            matches.append({
                a.queryIdx: (keypoints[0][a.queryIdx].pt, keypoints[target][a.trainIdx].pt)
                for a, b in matcher.knnMatch(descriptors[0], descriptors[target], k=2)
                if a.distance < .74 * b.distance
            })
        points = []
        for index in matches[0].keys() & matches[1].keys():
            (x, y), (rx, ry) = matches[0][index]
            if abs(y - ry) > 2 or not 14 < x - rx < 255:
                continue
            z = self.baseline * self.p[0, 0] / (x - rx)
            point = [(x - self.p[0, 2]) * z / self.p[0, 0],
                     (y - self.p[1, 2]) * z / self.p[1, 1], z]
            points.append((point, matches[1][index][1]))
        if len(points) < 14:
            return None
        world, pixels = (np.asarray(data) for data in zip(*points))
        solved, angle, translation, inliers = cv2.solvePnPRansac(
            world, pixels, self.p[:, :3].copy(), None, iterationsCount=75, reprojectionError=3.,
        )
        if not solved or inliers is None or len(inliers) < 12 or len(inliers) < .55 * len(points):
            return None
        spread = np.ptp(pixels[inliers.ravel()], axis=0)
        if spread[0] < 75 or spread[1] < 40:
            return None
        rotation = cv2.Rodrigues(angle)[0]
        offset = self.r.T @ (-rotation.T @ translation.ravel())
        forward = self.r.T @ rotation.T @ self.r @ np.array([0., 0., 1.])
        return {'right_m': float(offset[0]), 'forward_m': float(offset[2]),
                'yaw_right_deg': math.degrees(math.atan2(forward[0], forward[2])),
                'inliers': len(inliers)}
