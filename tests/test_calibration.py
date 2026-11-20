import copy
import tempfile
import unittest
from pathlib import Path
import yaml
from jetbot_stereo.calibration import load_calibration, validate_pair


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.left = dict(image_width=640, image_height=480,
                         camera_matrix={'data': [400., 0., 320., 0., 400., 240., 0., 0., 1.]},
                         rectification_matrix={'data': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
                         projection_matrix={'data': [400., 0., 320., 0., 0., 400., 240., 0., 0., 0., 1., 0.]})
        self.right = copy.deepcopy(self.left)
        self.right['projection_matrix']['data'][3] = -24.

    def test_metric_baseline(self):
        self.assertAlmostEqual(validate_pair(self.left, self.right), .06)

    def test_monocular_files_rejected(self):
        with self.assertRaises(ValueError):
            validate_pair(self.left, self.left)

    def test_swapped_cameras_rejected(self):
        with self.assertRaises(ValueError):
            validate_pair(self.right, self.left)

    def test_resolution_mismatch_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'left.yaml'
            path.write_text(yaml.safe_dump(self.left))
            self.assertEqual(load_calibration(path, 640, 480)['image_width'], 640)
            with self.assertRaises(ValueError):
                load_calibration(path, 1280, 720)

    def test_nonfinite_matrix_rejected(self):
        self.left['rectification_matrix']['data'][0] = float('nan')
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'left.yaml'
            path.write_text(yaml.safe_dump(self.left))
            with self.assertRaises(ValueError):
                load_calibration(path, 640, 480)


if __name__ == '__main__':
    unittest.main()
