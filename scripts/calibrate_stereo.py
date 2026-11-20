#!/usr/bin/env python3
"""Fit metric stereo calibration from stationary 8x6 checkerboard pairs."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np
import yaml

BOARD = (8, 6)


def corners(gray):
    found, points = cv2.findChessboardCornersSB(gray, BOARD, cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        found, points = cv2.findChessboardCorners(
            gray, BOARD, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            points = cv2.cornerSubPix(gray, points, (5, 5), (-1, -1),
                                     (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, .001))
    if not found:
        return None
    # This capture guide keeps board roll below 90 degrees. Canonicalize the
    # 180-degree ambiguity independently before pairing the cameras.
    if points[0, 0, 0] > points[-1, 0, 0]:
        points = points[::-1].copy()
    return points.astype(np.float32)


def collect(root):
    pairs, rejected, size = [], [], None
    folders = {p.parent for name in ('camera-0.jpg', 'camera-1.jpg') for p in root.rglob(name)}
    for folder in sorted(folders):
        left = folder/'camera-0.jpg'
        right = left.with_name('camera-1.jpg')
        imgs = [cv2.imread(str(p), cv2.IMREAD_GRAYSCALE) for p in (left, right)]
        reason = None
        status = left.with_name('status.txt')
        if status.exists() and status.read_text().strip() != '0':
            reason = '촬영 실패 로그'
        elif any(img is None for img in imgs):
            reason = '사진 누락 또는 읽기 실패'
        elif imgs[0].shape != imgs[1].shape:
            reason = '좌우 해상도 불일치'
        elif size is not None and imgs[0].shape[::-1] != size:
            reason = '촬영 간 해상도 변경'
        if reason:
            rejected.append(dict(folder=str(left.parent), reason=reason))
            continue
        points = [corners(img) for img in imgs]
        if any(p is None for p in points):
            rejected.append(dict(folder=str(left.parent), reason='한쪽 또는 양쪽 패턴 인식 실패'))
            continue
        size = imgs[0].shape[::-1]
        if any(max(float(np.sqrt(np.mean((a-b)**2))) for a, b in zip(points, old['points'])) < 5
               for old in pairs):
            rejected.append(dict(folder=str(left.parent), reason='이전 사진과 거의 같은 자세'))
            continue
        pairs.append(dict(folder=left.parent, points=points, images=imgs))
    return pairs, rejected, size


def delete_rejected_photos(root, rejected):
    """Remove only known photo filenames; preserve capture logs and reasons."""
    root = root.resolve()
    plans = []
    for item in rejected:
        folder = Path(item['folder']).resolve()
        if not folder.is_relative_to(root):
            raise ValueError('제외 사진 폴더가 지정한 촬영 폴더 밖에 있습니다')
        photos = [folder/name for name in ('camera-0.jpg', 'camera-1.jpg',
                                           'camera-0-detected.jpg', 'camera-1-detected.jpg')
                  if (folder/name).is_file() or (folder/name).is_symlink()]
        plans.append((item, folder, photos))
    for item, folder, photos in plans:
        record = dict(reason=item['reason'], timestamp_utc=datetime.now(timezone.utc).isoformat(),
                      photos_to_delete=[p.name for p in photos], deleted_files=[])
        audit = folder/'rejection.json'
        # Record why before deleting. If recording fails, leave the photos alone.
        audit.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
        for photo in photos:
            photo.unlink()
            record['deleted_files'].append(photo.name)
        audit.write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
        item['deleted_files'] = record['deleted_files']
    return sum(len(item['deleted_files']) for item in rejected)


def camera_yaml(name, size, k, d, r, p):
    def matrix(a):
        a = np.atleast_2d(a)
        return dict(rows=a.shape[0], cols=a.shape[1], data=a.astype(float).ravel().tolist())
    return dict(image_width=size[0], image_height=size[1], camera_name=name,
                distortion_model='plumb_bob', camera_matrix=matrix(k),
                distortion_coefficients=matrix(d.reshape(1, -1)),
                rectification_matrix=matrix(r), projection_matrix=matrix(p))


def fit(pairs, size, square_m, intrinsic_pairs=None):
    grid = np.zeros((BOARD[0]*BOARD[1], 3), np.float32)
    grid[:, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * square_m
    objects = [grid.copy() for _ in pairs]
    left = [p['points'][0] for p in pairs]
    right = [p['points'][1] for p in pairs]
    intrinsic_pairs = pairs if intrinsic_pairs is None else intrinsic_pairs
    intrinsic_objects = [grid.copy() for _ in intrinsic_pairs]
    rms0, k0, d0, rotations, _ = cv2.calibrateCamera(
        intrinsic_objects, [p['points'][0] for p in intrinsic_pairs], size, None, None)
    rms1, k1, d1, _, _ = cv2.calibrateCamera(
        intrinsic_objects, [p['points'][1] for p in intrinsic_pairs], size, None, None)
    rms, k0, d0, k1, d1, rotation, translation, _, _ = cv2.stereoCalibrate(
        objects, left, right, k0, d0, k1, d1, size,
        flags=cv2.CALIB_FIX_INTRINSIC,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7))
    r0, r1, p0, p1, q, _, _ = cv2.stereoRectify(
        k0, d0, k1, d1, size, rotation, translation,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    vertical = []
    for a, b in zip(left, right):
        ar = cv2.undistortPoints(a, k0, d0, R=r0, P=p0)
        br = cv2.undistortPoints(b, k1, d1, R=r1, P=p1)
        vertical.extend(np.abs(ar[:, 0, 1] - br[:, 0, 1]).tolist())
    normals = [cv2.Rodrigues(v)[0][:, 2] for v in rotations]
    tilt_span = max(math.degrees(math.acos(float(np.clip(np.dot(a, b), -1, 1))))
                    for a in normals for b in normals)
    report = dict(pair_count=len(pairs), intrinsic_pair_count=len(intrinsic_pairs),
                  image_size=list(size), square_size_m=square_m,
                  monocular_rms_px=[float(rms0), float(rms1)], stereo_rms_px=float(rms),
                  baseline_m=float(np.linalg.norm(translation)),
                  rectified_baseline_m=float(-p1[0, 3]/p1[0, 0]),
                  vertical_error_mean_px=float(np.mean(vertical)),
                  vertical_error_p95_px=float(np.percentile(vertical, 95)),
                  board_normal_span_degrees=tilt_span,
                  rotation=rotation.tolist(), translation_m=translation.ravel().tolist(),
                  note='Errors are measured on fitting images, not an independent validation set.')
    cameras = [camera_yaml('stereo_'+s, size, k, d, r, p)
               for s, k, d, r, p in [('left', k0, d0, r0, p0), ('right', k1, d1, r1, p1)]]
    return report, cameras, [(k0,d0,r0,p0), (k1,d1,r1,p1)], q


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', type=Path)
    parser.add_argument('--square-mm', type=float, default=None,
                        help='Square size; use session config, or legacy 121/7 mm if absent')
    parser.add_argument('--expected-baseline-mm', type=float, default=57)
    parser.add_argument('--intrinsics-dataset', type=Path,
                        help='Reuse lens calibration views; fit stereo pose only from the main dataset')
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--keep-rejected', action='store_true',
                        help='Keep rejected photos; by default remove both photos of rejected pairs')
    parser.add_argument('--require-latest', action='store_true',
                        help='With --check-only, fail unless the latest pair is accepted')
    args = parser.parse_args()
    if not args.dataset.is_dir():
        parser.error('촬영 폴더가 없습니다')
    if args.square_mm is None:
        config = args.dataset/'capture-config.json'
        args.square_mm = float(json.loads(config.read_text())['square_mm']) if config.exists() else 121/7
    if not all(math.isfinite(v) and v > 0 for v in (args.square_mm, args.expected_baseline_mm)):
        parser.error('칸 크기와 기준선은 양수여야 합니다')
    # Capture the latest folder BEFORE cleanup so a deleted failed pair cannot
    # make an older successful pair look like the just-captured one.
    folders = sorted({p.parent for name in ('camera-0.jpg', 'camera-1.jpg')
                      for p in args.dataset.rglob(name)})
    pairs, rejected, size = collect(args.dataset)
    latest_accepted = bool(folders and folders[-1] in [p['folder'] for p in pairs])
    print(f'유효한 서로 다른 자세: {len(pairs)}쌍 / 제외: {len(rejected)}쌍')
    for item in rejected:
        print(f'제외: {Path(item["folder"]).name}: {item["reason"]}')
    if rejected and not args.keep_rejected:
        count = delete_rejected_photos(args.dataset, rejected)
        print(f'제외 사진 {count}개 삭제 완료. 촬영 로그와 rejection.json은 보존합니다.')
    if args.check_only:
        if args.require_latest:
            if not latest_accepted:
                return 1
            print('방금 촬영한 양쪽 패턴 인식 성공.')
        return 0
    intrinsic_pairs = None
    if args.intrinsics_dataset is not None:
        intrinsic_pairs, _, intrinsic_size = collect(args.intrinsics_dataset)
        if intrinsic_size != size or len(intrinsic_pairs)<12 or len(pairs)<3:
            print('렌즈 보정용 12쌍과 좌우 관계 계산용 3쌍 이상, 동일한 해상도가 필요합니다.',file=sys.stderr)
            return 2
        config=args.intrinsics_dataset/'capture-config.json'
        if config.exists() and abs(float(json.loads(config.read_text())['square_mm'])-args.square_mm)>1e-6:
            parser.error('기존 렌즈 보정 자료와 현재 패턴의 칸 크기가 다릅니다')
    elif len(pairs) < 12:
        print('보정에는 서로 다른 자세의 유효 사진이 최소 12쌍 필요합니다. 목표는 20쌍입니다.', file=sys.stderr)
        return 2
    report, cameras, params, q = fit(pairs, size, args.square_mm/1000, intrinsic_pairs)
    reasons = []
    if not all(math.isfinite(report[k]) for k in
               ('stereo_rms_px', 'baseline_m', 'vertical_error_mean_px', 'vertical_error_p95_px')):
        reasons.append('계산 결과에 비정상 값')
    if report['stereo_rms_px'] > 1.5 or max(report['monocular_rms_px']) > 1.5:
        reasons.append('재투영 오차가 1.5 px 초과')
    if report['vertical_error_mean_px'] > 1 or report['vertical_error_p95_px'] > 2:
        reasons.append('정렬 후 수직 오차가 큼')
    if abs(report['baseline_m']*1000-args.expected_baseline_mm) > args.expected_baseline_mm*.25:
        reasons.append(f'계산된 카메라 간격이 실측 {args.expected_baseline_mm:g} mm 기준과 25% 이상 다름')
    if report['rectified_baseline_m'] <= 0 or abs(cameras[1]['projection_matrix']['data'][7]) > 1e-6:
        reasons.append('좌우 카메라 순서 또는 수평 스테레오 배치 확인 필요')
    if report['board_normal_span_degrees'] < 10:
        reasons.append('패턴 기울기 변화가 부족함 (10도 미만)')
    report.update(accepted=not reasons, rejection_reasons=reasons,
                  expected_baseline_mm=args.expected_baseline_mm,
                  used_folders=[str(p['folder'].resolve()) for p in pairs], excluded=rejected)
    if args.intrinsics_dataset is not None:
        report['intrinsics_dataset']=str(args.intrinsics_dataset.resolve())
    output = args.dataset/'calibration-result'
    if output.exists():
        print(f'결과 폴더가 이미 존재합니다. 기존 결과를 보존합니다: {output}', file=sys.stderr)
        return 2
    output.mkdir()
    (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(f'간격 {report["baseline_m"]*1000:.2f} mm, stereo RMS {report["stereo_rms_px"]:.3f} px, '
          f'수직 오차 평균 {report["vertical_error_mean_px"]:.3f} px')
    if reasons:
        print('보정 품질 확인 실패: '+'; '.join(reasons), file=sys.stderr)
        print(f'진단 보고서: {output}/report.json')
        return 2
    for side, data in zip(('left', 'right'), cameras):
        (output/(side+'.yaml')).write_text(yaml.safe_dump(data, sort_keys=False))
    np.save(output/'Q.npy', q)
    for index, pair in enumerate(pairs[:3]):
        rects = []
        for image, (k,d,r,p) in zip(pair['images'], params):
            mx, my = cv2.initUndistortRectifyMap(k,d,r,p,size,cv2.CV_32FC1)
            rects.append(cv2.remap(image,mx,my,cv2.INTER_LINEAR))
        preview = cv2.cvtColor(np.hstack(rects), cv2.COLOR_GRAY2BGR)
        for y in range(20,size[1],40):
            cv2.line(preview,(0,y),(size[0]*2-1,y),(0,200,0),1)
        cv2.imwrite(str(output/f'rectified-{index}.jpg'),preview)
    print(f'보정 파일 생성: {output}/left.yaml, right.yaml')
    print('검토 후 이 파일을 프로젝트 calibration/ 폴더로 복사하세요. 기존 값은 자동으로 덮어쓰지 않습니다.')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (cv2.error, ValueError) as exc:
        print('보정 계산 실패: '+str(exc), file=sys.stderr)
        sys.exit(2)
