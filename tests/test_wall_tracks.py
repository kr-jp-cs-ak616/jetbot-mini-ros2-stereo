import json,math
import numpy as np
from jetbot_stereo.wall_tracks import WallTracks,RelativeHeading
from jetbot_stereo.boundary_walls import SparseWallMap


def wall(a,b,h=.1):return dict(endpoints_xy_m=[a,b],height_m=h,floor_z_m=-.099)

def test_repeated_wall_with_noise_keeps_one_id_across_old_quantization_bins():
 t=WallTracks();rng=np.random.default_rng(2)
 for i in range(80):
  y=1+rng.normal(0,.012);t.observe([wall([-.5,y],[.5,y+rng.normal(0,.008)])],i*.2)
 assert len(t.confirmed())==1 and t.confirmed()[0]['id']==0
 assert t.confirmed()[0]['observations']==80
 json.dumps(t.confirmed())

def test_same_wall_after_camera_rotation_uses_same_id():
 t=WallTracks();world=np.array([[-.4,1.],[.4,1.]])
 for i,angle in enumerate(np.linspace(0,math.pi/2,40)):
  r=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
  camera=(world-np.array([.02,0]))@r
  recovered=camera@r.T+np.array([.02,0])
  t.observe([wall(*recovered)],i*.2)
 assert len(t.confirmed())==1 and t.confirmed()[0]['id']==0
 assert np.allclose(t.confirmed()[0]['endpoints_xy_m'],world,atol=1e-6)

def test_distinct_parallel_walls_corner_and_gap_are_not_merged():
 t=WallTracks()
 observations=[wall([0,1],[1,1]),wall([0,1.3],[1,1.3]),wall([1,1],[1,2]),wall([2,1],[3,1])]
 for i in range(4):t.observe(observations,i*.2)
 assert len(t.confirmed())==4

def test_fragments_extend_existing_wall_and_bridge_without_duplicate_id():
 t=WallTracks()
 for i in range(3):t.observe([wall([0,1],[.4,1]),wall([.7,1],[1,1])],i*.2)
 assert len(t.confirmed())==2
 t.observe([wall([.3,1],[.8,1])],.8)
 assert len(t.confirmed())==1 and t.confirmed()[0]['id']==0
 assert np.allclose(t.confirmed()[0]['endpoints_xy_m'],[[0,1],[1,1]])

def test_rebuilding_cells_removes_old_wall_trail():
 t=WallTracks()
 for i in range(3):t.observe([wall([0,1],[1,1])],i*.2)
 for i in range(30):t.observe([wall([0,1.05],[1,1.05])],1+i*.2)
 m=SparseWallMap()
 for s in t.confirmed():m.add_segment(*s['endpoints_xy_m'])
 assert all(c[1]>50 for c in m.cells)

def test_same_frame_cannot_confirm_wall_or_grow_unbounded_tracks():
 t=WallTracks(max_tracks=3)
 for _ in range(10):t.observe([wall([0,1],[1,1])],1.)
 assert t.confirmed()==[]
 for i in range(20):t.observe([wall([0,2+i],[1,2+i])],2+i*.01)
 assert len(t.tracks)<=3

def test_heading_wrap_and_rotation_direction():
 h=RelativeHeading();h.update(math.radians(179));r=h.update(math.radians(-179))
 assert abs(r['relative_yaw_deg']-2)<1e-6 and r['rotation_direction']=='left'
 r=h.update(math.radians(178));assert abs(r['relative_yaw_deg']+1)<1e-6 and r['rotation_direction']=='right'

def test_visual_pose_reset_is_not_accepted_as_new_wall_location():
 from jetbot_stereo.wall_tracks import PoseContinuity
 g=PoseContinuity();assert g.accept(1.,[2.,0,0],0.)
 assert g.accept(1.2,[2.01,0,0],.1)
 assert not g.accept(1.4,[0.,0,0],0.)
 assert g.accept(1.6,[2.02,0,0],.15)

def test_visual_pose_turn_across_pi_is_continuous():
 from jetbot_stereo.wall_tracks import PoseContinuity
 g=PoseContinuity();assert g.accept(1.,[0,0,0],math.radians(179))
 assert g.accept(1.2,[0,0,0],math.radians(-179))
def test_scan_confirmation_uses_distinct_current_observations_in_odom_frame():
    import numpy as np
    from jetbot_stereo.wall_scan import WallScanConfirmation

    gate = WallScanConfirmation()
    for i in range(3):
        angle = .1*i
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0],
                             [np.sin(angle), np.cos(angle), 0], [0, 0, 1.]])
        translation = np.array([.06*i, .02*i, 0])
        endpoints = (np.array([[.72, -.25, 0], [.72, .25, 0]]) - translation) @ rotation
        optical = np.column_stack((-endpoints[:, 1], -endpoints[:, 2], endpoints[:, 0]))
        wall = {'floor_endpoints_optical': optical}
        confirmed = gate.confirm([wall], 1.+.2*i, rotation, translation)
        assert len(confirmed) == int(i == 2)
        assert not gate.confirm([wall], 1.+.2*i, rotation, translation)
    shifted = {'floor_endpoints_optical': optical + [0, 0, .6]}
    assert not gate.confirm([shifted], 1.6, rotation, translation)
    assert gate.confirm([wall], 1.8, rotation, translation)
    assert gate.confirm([wall], 2.0, rotation, translation)
    assert not gate.confirm([wall], 2.8, rotation, translation)
    gate.clear()
    assert not gate.confirm([wall], 3.0, rotation, translation)
