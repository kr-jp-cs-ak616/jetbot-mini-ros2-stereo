import importlib.util
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from measure_stereo import measure
from calibrate_stereo import camera_yaml


class DistanceTests(unittest.TestCase):
    def setUp(self):
        self.k=np.array([[500.,0,320.],[0,500.,240.],[0,0,1.]])
        self.p0=np.column_stack((self.k,np.zeros(3)))
        self.p1=self.p0.copy()
        self.p1[0,3]=-500*.057
        self.cameras=[camera_yaml(side,(640,480),self.k,np.zeros(5),np.eye(3),p)
                      for side,p in [('left',self.p0),('right',self.p1)]]
        self.grid=np.zeros((48,3),np.float32)
        self.grid[:,:2]=np.mgrid[:8,:6].T.reshape(-1,2)*.025
        self.t=np.array([-.0875,-.0625,.6])
        self.points=[cv2.projectPoints(self.grid,np.zeros(3),self.t+offset,self.k,None)[0]
                     for offset in (np.zeros(3),np.array([-.057,0,0]))]

    def test_known_60cm_depth_and_range(self):
        report=measure(self.points,self.cameras,25.)
        self.assertAlmostEqual(report['forward_depth_m'],.6,places=5)
        self.assertAlmostEqual(report['straight_line_range_m'],.6,places=5)
        self.assertAlmostEqual(report['reconstructed_square_mm'],25.,places=3)

    def test_rejects_old_pattern_scale(self):
        with self.assertRaises(ValueError):
            measure(self.points,self.cameras,121/7)

    def test_rejects_vertical_mismatch(self):
        self.points[1][:,:,1]+=5
        with self.assertRaises(ValueError):
            measure(self.points,self.cameras,25.)
