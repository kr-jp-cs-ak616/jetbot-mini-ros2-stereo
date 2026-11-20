import cv2,numpy as np,pytest
from jetbot_stereo.boundary_walls import model_partitions,wall_geometry,SparseWallMap

P=np.array([[400.,0.,320.,0.],[0.,400.,240.,0.],[0.,0.,1.,0.]])

def test_floor_boundary_sets_metric_wall_and_excludes_background():
 pair={'x0':100,'x1':540,'top':{'m':0.,'b':250.},'bottom':{'m':0.,'b':350.}}
 wall,_=wall_geometry(pair,P,np.eye(3),.1)
 assert wall['distance_m']==pytest.approx(400*.1/(350-240))
 assert wall['height_m']==pytest.approx(.1-10*.1/110)
 assert wall['metric_verified'] is False

def test_horizon_boundary_is_rejected():
 pair={'x0':100,'x1':540,'top':{'m':0.,'b':200.},'bottom':{'m':0.,'b':240.}}
 with pytest.raises(ValueError):wall_geometry(pair,P,np.eye(3),.1)

def test_preprocessing_masks_background_and_emits_only_wall_depth():
 image=np.full((480,640,3),55,np.uint8);image[350:]=100
 cv2.rectangle(image,(20,250),(620,349),(180,180,180),-1)
 m=model_partitions(image,P,np.eye(3),.1)
 assert m['walls']
 assert np.all(m['masked_rgb'][100]==0)
 assert m['depth'][300,320]>0
 assert m['depth'][400,320]==0
 assert np.all(m['depth'][100]==0)

def test_blank_scene_does_not_invent_wall():
 m=model_partitions(np.full((480,640,3),180,np.uint8),P,np.eye(3),.1)
 assert m['walls']==[] and not m['depth'].any() and not m['masked_rgb'].any()

def test_known_partition_height_accepts_an_upper_edge_near_the_image_top():
 image=np.full((480,640,3),55,np.uint8);image[390:]=100
 cv2.rectangle(image,(20,70),(620,389),(180,180,180),-1)
 assert not model_partitions(image,P,np.eye(3),.094)['walls']
 walls=model_partitions(image,P,np.eye(3),.094,partition_height_m=.2)['walls']
 assert len(walls)==1
 assert walls[0]['distance_m']==pytest.approx(.25,abs=.01)
 assert not model_partitions(image,P,np.eye(3),.094,partition_height_m=.3)['walls']

def test_near_partition_recovers_a_faint_floor_edge_with_bounded_pitch_fit():
 projection=np.array([[880.,0.,453.,0.],[0.,880.,235.,0.],[0.,0.,1.,0.]])
 image=np.full((480,640,3),45,np.uint8)
 floor=np.random.default_rng(41).normal(122,9,(205,640))
 image[275:]=np.clip(floor,0,255).astype(np.uint8)[:,:,None]
 cv2.rectangle(image,(30,30),(620,274),(130,130,130),-1)
 args=(image,projection,np.eye(3),.1,3.5)
 assert not model_partitions(*args,partition_height_m=.199)['walls']
 wall,=model_partitions(*args,partition_height_m=.199,partition_pitch_tolerance_deg=3.)['walls']
 assert wall['distance_m']==pytest.approx(.72,abs=.02)
 assert wall['pitch_deg']==pytest.approx(5.3,abs=.3)
 assert wall['metric_source']=='known_partition_height_and_floor_boundary'
 assert wall['metric_verified'] is False
 assert not model_partitions(*args,partition_height_m=.199,partition_pitch_tolerance_deg=1.)['walls']

def test_sparse_map_has_no_cells_behind_wall():
 m=SparseWallMap(.02);m.add_segment([-.5,1.],[.5,1.])
 assert m.cells and {cell[1] for cell in m.cells}=={50}
 assert len(m.cells)<100 and m.serialize()['free_space_inferred'] is False
 import json
 assert json.loads(json.dumps(m.serialize()))['occupied_cells']

def test_sparse_memory_limit():
 m=SparseWallMap(.02,max_cells=2)
 with pytest.raises(ValueError):m.add_segment([0,0],[1,0])

@pytest.mark.parametrize("height,pitch",[(float('nan'),0.),(.1,float('nan'))])
def test_invalid_mounting_geometry_is_not_serialized(height,pitch):
 pair={'x0':100,'x1':540,'top':{'m':0.,'b':250.},'bottom':{'m':0.,'b':350.}}
 with pytest.raises(ValueError):wall_geometry(pair,P,np.eye(3),height,pitch)

def test_delayed_odometry_matches_original_frame_not_newest_frame():
 from jetbot_stereo.boundary_walls import ObservationBuffer
 b=ObservationBuffer();b.add(10.,'old frame walls');b.add(10.5,'new frame walls')
 assert b.match(10.)==(10.,'old frame walls')
 assert b.match(10.2) is None
 assert b.match(10.5)==(10.5,'new frame walls')

def test_observation_memory_is_bounded():
 from jetbot_stereo.boundary_walls import ObservationBuffer
 b=ObservationBuffer(3)
 for i in range(10):b.add(float(i),i)
 assert len(b.items)==3 and b.match(0.) is None
 assert b.match(9.)==(9.,9)
