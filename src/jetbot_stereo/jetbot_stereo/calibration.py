"""Load ROS camera calibration YAML without inventing intrinsics or baseline."""
import math
from pathlib import Path
import yaml


def load_calibration(path, width, height):
    with Path(path).open() as stream:
        data = yaml.safe_load(stream)
    if (data['image_width'], data['image_height']) != (width, height):
        raise ValueError('Calibration resolution does not match capture resolution')
    for key, length in [('camera_matrix', 9), ('rectification_matrix', 9),
                        ('projection_matrix', 12)]:
        values = data[key]['data']
        if len(values) != length or not all(math.isfinite(float(v)) for v in values):
            raise ValueError('Invalid calibration matrix: ' + key)
    if data['camera_matrix']['data'][0] <= 0 or data['projection_matrix']['data'][0] <= 0:
        raise ValueError('Invalid focal length')
    return data


def validate_pair(left, right):
    lp, rp = left['projection_matrix']['data'], right['projection_matrix']['data']
    baseline = lp[3] / lp[0] - rp[3] / rp[0]
    if not math.isfinite(baseline) or baseline <= 0:
        raise ValueError('Positive stereo baseline required; use joint stereo calibration')
    if abs(lp[7]) > 1e-6 or abs(rp[7]) > 1e-6:
        raise ValueError('This pipeline requires a horizontal rectified stereo pair')
    return baseline
