#!/usr/bin/env python3
"""Measure a stationary checkerboard center using calibrated stereo geometry."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import yaml
from calibrate_stereo import BOARD, corners


def measure(points, cameras, square_mm):
    rectified=[]
    matrices=[]
    rotations=[]
    for observed, camera in zip(points, cameras):
        k=np.array(camera['camera_matrix']['data'],dtype=float).reshape(3,3)
        d=np.array(camera['distortion_coefficients']['data'],dtype=float)
        r=np.array(camera['rectification_matrix']['data'],dtype=float).reshape(3,3)
        p=np.array(camera['projection_matrix']['data'],dtype=float).reshape(3,4)
        rectified.append(cv2.undistortPoints(observed,k,d,R=r,P=p).reshape(-1,2))
        matrices.append(p)
        rotations.append(r)
    vertical=np.abs(rectified[0][:,1]-rectified[1][:,1])
    if np.mean(vertical)>1 or np.percentile(vertical,95)>2:
        raise ValueError('좌우 정렬 오차가 큽니다. 보정/카메라 장착/촬영 시점 확인 필요')
    homogeneous=cv2.triangulatePoints(matrices[0],matrices[1],rectified[0].T,rectified[1].T)
    if np.any(np.abs(homogeneous[3])<1e-9):
        raise ValueError('안정적인 시차를 얻지 못했습니다')
    cloud=(homogeneous[:3]/homogeneous[3]).T
    if not np.isfinite(cloud).all() or np.any(cloud[:,2]<=0):
        raise ValueError('삼각측량 결과가 유효하지 않습니다. 좌우 순서를 확인하세요')
    # Rectification is a rotation around the left camera center. Undo it to
    # report forward depth and range in the original left optical frame.
    original=cloud@rotations[0]
    grid=original.reshape(BOARD[1],BOARD[0],3)
    edges=np.concatenate((np.linalg.norm(np.diff(grid,axis=0),axis=2).ravel(),
                          np.linalg.norm(np.diff(grid,axis=1),axis=2).ravel()))
    median_square_mm=float(np.median(edges)*1000)
    if abs(median_square_mm-square_mm)>square_mm*.15:
        raise ValueError(f'복원한 한 칸이 {median_square_mm:.2f} mm입니다. '
                         f'현재 패턴 {square_mm:g} mm 또는 보정 거리 척도를 확인하세요')
    center=np.mean(original,axis=0)
    return dict(target='checkerboard center', reference='left camera optical center',
                forward_depth_m=float(center[2]),
                straight_line_range_m=float(np.linalg.norm(center)),
                center_xyz_m=center.tolist(),
                reconstructed_square_mm=median_square_mm,
                expected_square_mm=square_mm,
                vertical_error_mean_px=float(np.mean(vertical)),
                note='Calibration-based estimate; compare against a ruler for independent accuracy validation.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pair',type=Path)
    parser.add_argument('--calibration-dir',type=Path,required=True)
    parser.add_argument('--square-mm',type=float,default=25.)
    args=parser.parse_args()
    if not np.isfinite(args.square_mm) or args.square_mm<=0:
        parser.error('칸 크기는 양수여야 합니다')
    cameras=[yaml.safe_load((args.calibration_dir/(side+'.yaml')).read_text())
             for side in ('left','right')]
    points=[]
    for sensor,camera in enumerate(cameras):
        image=cv2.imread(str(args.pair/f'camera-{sensor}.jpg'),cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f'카메라 {sensor} 사진이 없습니다')
        if image.shape[::-1]!=(camera['image_width'],camera['image_height']):
            raise ValueError('보정 해상도와 촬영 해상도가 다릅니다')
        detected=corners(image)
        if detected is None:
            raise ValueError(f'카메라 {sensor}에서 8x6 패턴이 인식되지 않았습니다')
        points.append(detected)
    result=measure(points,cameras,args.square_mm)
    (args.pair/'distance.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(f'왼쪽 렌즈 중심 → 패턴 중심 직선거리: {result["straight_line_range_m"]*100:.1f} cm')
    print(f'왼쪽 카메라 전방 깊이(Z): {result["forward_depth_m"]*100:.1f} cm')
    print('보정 사진에서 계산한 추정값입니다. 자로 잰 거리와 비교해 정확도를 확인하세요.')
    return 0


if __name__=='__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, cv2.error) as exc:
        print('거리 계산 불가: '+str(exc),file=sys.stderr)
        sys.exit(2)
