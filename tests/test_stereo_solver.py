"""Metric geometry test using synthetic observations with known ground truth."""
import importlib.util
from pathlib import Path
import unittest
import tempfile
from unittest.mock import patch
import json
import cv2
import numpy as np

spec = importlib.util.spec_from_file_location(
    'stereo_solver', Path(__file__).resolve().parents[1]/'scripts/calibrate_stereo.py')
solver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(solver)


class StereoSolverTests(unittest.TestCase):
    def test_cleanup_removes_only_rejected_photos(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            good=root/'001-good'
            bad=root/'002-bad'
            good.mkdir(); bad.mkdir()
            for folder in (good,bad):
                for name in ('camera-0.jpg','camera-1.jpg','camera-0-detected.jpg',
                             'camera-1-detected.jpg','camera-0.log','status.txt','unrelated.jpg'):
                    (folder/name).write_text('fixture')
            rejected=[dict(folder=str(bad),reason='pattern not detected')]
            self.assertEqual(solver.delete_rejected_photos(root,rejected),4)
            self.assertTrue((good/'camera-0.jpg').exists())
            self.assertTrue((good/'camera-1.jpg').exists())
            self.assertTrue((bad/'camera-0.log').exists())
            self.assertTrue((bad/'unrelated.jpg').exists())
            self.assertFalse((bad/'camera-0.jpg').exists())
            record=json.loads((bad/'rejection.json').read_text())
            self.assertEqual(record['reason'],'pattern not detected')
            self.assertEqual(len(record['deleted_files']),4)

    def test_cleanup_refuses_outside_dataset(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)/'dataset'; root.mkdir()
            other=Path(directory)/'other'; other.mkdir()
            photo=other/'camera-0.jpg'; photo.write_text('fixture')
            with self.assertRaises(ValueError):
                solver.delete_rejected_photos(root,[dict(folder=str(other),reason='rejected')])
            self.assertTrue(photo.exists())

    def test_failed_latest_still_fails_after_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            good=root/'001-good'; good.mkdir()
            bad=root/'002-bad'; bad.mkdir()
            for folder in (good,bad):
                for name in ('camera-0.jpg','camera-1.jpg'):
                    (folder/name).write_text('fixture')
            with patch.object(solver,'collect',return_value=(
                    [dict(folder=good)], [dict(folder=str(bad),reason='rejected')], (640,480))), \
                 patch('sys.argv',['calibrate_stereo.py',str(root),'--check-only','--require-latest']):
                self.assertEqual(solver.main(),1)
            self.assertFalse((bad/'camera-0.jpg').exists())
            self.assertTrue((good/'camera-0.jpg').exists())

    def test_recovers_metric_baseline_and_rectifies(self):
        rng = np.random.default_rng(42)
        square = .121/7
        grid = np.zeros((48,3), np.float32)
        grid[:,:2] = np.mgrid[:8,:6].T.reshape(-1,2)*square
        k = np.array([[520.,0.,320.],[0.,525.,240.],[0.,0.,1.]])
        distortion = np.array([-.04,.006,.0002,.0001,0.])
        stereo_r = cv2.Rodrigues(np.array([.005,.02,-.01]))[0]
        stereo_t = np.array([[-.057],[.0007],[.001]])
        pairs=[]
        for _ in range(20):
            rotation = rng.uniform(-.45,.45,3)
            translation = np.array([[rng.uniform(-.12,.02)],
                                    [rng.uniform(-.09,.02)],
                                    [rng.uniform(.38,.7)]])
            right_r = cv2.Rodrigues(stereo_r@cv2.Rodrigues(rotation)[0])[0]
            right_t = stereo_r@translation+stereo_t
            points=[]
            for r,t in ((rotation,translation),(right_r,right_t)):
                projected=cv2.projectPoints(grid,r,t,k,distortion)[0]
                points.append((projected+rng.normal(0,.03,projected.shape)).astype(np.float32))
            pairs.append(dict(points=points))
        report,cameras,_,_=solver.fit(pairs,(640,480),square)
        self.assertAlmostEqual(report['baseline_m'], np.linalg.norm(stereo_t), delta=.002)
        self.assertGreater(report['rectified_baseline_m'],0)
        self.assertLess(report['vertical_error_mean_px'],.2)
        self.assertLess(report['stereo_rms_px'],.3)
        self.assertEqual(cameras[1]['projection_matrix']['cols'],4)
        self.assertEqual(cameras[0]['image_width'],640)
        split_report,_,_,_=solver.fit(pairs[-5:],(640,480),square,intrinsic_pairs=pairs)
        self.assertEqual(split_report['pair_count'],5)
        self.assertEqual(split_report['intrinsic_pair_count'],20)
        self.assertAlmostEqual(split_report['baseline_m'],np.linalg.norm(stereo_t),delta=.002)
        self.assertLess(split_report['vertical_error_mean_px'],.2)


if __name__=='__main__':
    unittest.main()
