import numpy as np
from jetbot_stereo.map_view import wall_map_center

def test_view_centers_on_wall_map_after_robot_moves_away_from_origin():
 center=wall_map_center([{'endpoints_xy_m':[[2.,.5],[3.4,1.]],'height_m':.2,'floor_z_m':-.1}])
 assert np.allclose(center,[2.7,.75,0.])

def test_empty_or_invalid_geometry_has_safe_origin_view():
 assert wall_map_center([])==[0.,0.,0.]
 assert wall_map_center([{'endpoints_xy_m':[[float('nan'),0],[1,1]]}])==[0.,0.,0.]
