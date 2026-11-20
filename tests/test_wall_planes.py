import importlib.util
from pathlib import Path
import numpy as np

spec=importlib.util.spec_from_file_location('walls',Path(__file__).resolve().parents[1]/'scripts/detect_wall_planes.py')
w=importlib.util.module_from_spec(spec);spec.loader.exec_module(w)

def test_vertical_plane_with_noise_has_metric_distance():
 r=np.random.default_rng(3)
 p=np.column_stack([r.uniform(-1,1,700),r.uniform(-.8,.8,700),r.normal(2,.004,700)])
 planes=w.fit_vertical_planes(p)
 assert planes and abs(planes[0]['perpendicular_distance_m']-2)<.02
 assert planes[0]['semantic_wall_confirmed'] is False

def test_horizontal_floor_is_not_reported_as_vertical_plane():
 r=np.random.default_rng(2)
 p=np.column_stack([r.uniform(-1,1,700),r.normal(.3,.002,700),r.uniform(.5,3,700)])
 assert w.fit_vertical_planes(p)==[]

def test_missing_depth_does_not_invent_wall():
 assert w.fit_vertical_planes(np.empty((0,3)))==[]

def test_stereo_known_disparity_recovers_depth_and_blank_stays_unknown(tmp_path):
 import cv2,yaml
 cv2.setNumThreads(2)
 k=[400.,0.,320.,0.,400.,240.,0.,0.,1.]
 for side,tx in [('left',0.),('right',-40.)]:
  data={'camera_matrix':{'data':k},'distortion_coefficients':{'data':[0.]*5},'rectification_matrix':{'data':[1.,0.,0.,0.,1.,0.,0.,0.,1.]},'projection_matrix':{'data':[400.,0.,320.,tx,0.,400.,240.,0.,0.,0.,1.,0.]}}
  (tmp_path/(side+'.yaml')).write_text(yaml.safe_dump(data))
 r=np.random.default_rng(11);gray=r.integers(0,256,(480,640),dtype=np.uint8)
 left=cv2.cvtColor(gray,cv2.COLOR_GRAY2BGR);right=np.zeros_like(left);right[:,:-48]=left[:,48:]
 result,_=w.analyze_pair(left,right,tmp_path)
 assert result['valid_depth_fraction']>.5
 assert result['planes'] and abs(result['planes'][0]['perpendicular_distance_m']-400*.1/48)<.03
 blank=np.full_like(left,180)
 result,_=w.analyze_pair(blank,blank,tmp_path)
 assert result['valid_depth_fraction']==0 and result['planes']==[]
 assert result['unknown_is_free_space'] is False
