import importlib.util
from pathlib import Path
import numpy as np,pytest

spec=importlib.util.spec_from_file_location('world_to_map',Path(__file__).resolve().parents[1]/'scripts/world_to_occupancy.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def box(x0,y0,x1,y1):return {'polygon':[[x0,y0],[x1,y0],[x1,y1],[x0,y1]]}

def pixel(image,origin,x,y,res=.025):
 col=int(np.floor((x-origin[0])/res));row=image.shape[0]-1-int(np.floor((y-origin[1])/res))
 return image[row,col]

def test_enclosure_exterior_and_vertical_origin_are_preserved():
 boxes=[box(0,0,2,.1),box(0,1.9,2,2),box(0,0,.1,2),box(1.9,0,2,2),box(.8,1.4,1.,1.6)]
 image,origin,_=m.rasterize(boxes)
 assert pixel(image,origin,1,1)==254
 assert pixel(image,origin,-.1,1)==205
 assert pixel(image,origin,.9,1.5)==0
 assert pixel(image,origin,.9,.5)==254

def test_open_wall_does_not_invent_enclosed_free_space():
 image,_,_=m.rasterize([box(0,0,1,.05)])
 assert not np.any(image==254) and np.any(image==205)

def test_visual_boxes_are_not_double_counted(tmp_path):
 path=tmp_path/'room.world'
 path.write_text('<sdf><world name="r"><model name="walls"><static>true</static><pose>1 2 0 0 0 1.5707963267948966</pose><link name="l"><collision name="c"><pose>1 0 .2 0 0 0</pose><geometry><box><size>1 .1 .4</size></box></geometry></collision><visual name="v"><geometry><box><size>1 .1 .4</size></box></geometry></visual></link></model></world></sdf>')
 name,boxes=m.boxes_from_world(path)
 assert name=='r' and len(boxes)==1
 assert np.allclose(np.mean(boxes[0]['polygon'],axis=0),[1,3])
