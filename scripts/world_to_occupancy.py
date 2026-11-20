#!/usr/bin/env python3
"""Convert static, horizontal SDF box collisions to a conservative ROS 2D map.

Only the restricted geometry used by map2_room.world is supported. Plugins and
includes are not executed. Open exterior remains unknown; enclosed space is free.
"""
import argparse,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2,numpy as np,yaml


def pose(element):
    text=element.findtext('pose','0 0 0 0 0 0');values=list(map(float,text.split()))
    if len(values)!=6 or not all(math.isfinite(x) for x in values):raise ValueError('Invalid SDF pose')
    if element.find('pose') is not None and element.find('pose').attrib.get('relative_to'):raise ValueError('relative_to is not supported')
    x,y,z,roll,pitch,yaw=values
    if abs(roll)>1e-8 or abs(pitch)>1e-8:raise ValueError('Tilted boxes are not supported')
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,x],[s,c,y],[0,0,1.]]),z


def boxes_from_world(path):
    root=ET.parse(path).getroot();world=root.find('world')
    if world is None:raise ValueError('SDF world is required')
    boxes=[]
    for model in world.findall('model'):
        if model.findtext('static','false').strip().lower() not in ('true','1'):raise ValueError('Dynamic models require separate treatment')
        if model.findall('model'):raise ValueError('Nested models are not supported')
        model_t,model_z=pose(model)
        for link in model.findall('link'):
            link_t,link_z=pose(link)
            for collision in link.findall('collision'):
                size=collision.findtext('geometry/box/size')
                if size is None:raise ValueError('Non-box collision geometry is not supported')
                dims=list(map(float,size.split()))
                if len(dims)!=3 or not all(math.isfinite(v) and v>0 for v in dims):raise ValueError('Invalid box dimensions')
                t,z=pose(collision);t=model_t@link_t@t;z+=model_z+link_z
                if z+dims[2]/2<=0:continue
                sx,sy,sz=dims
                points=np.array([[-sx/2,-sy/2,1],[sx/2,-sy/2,1],[sx/2,sy/2,1],[-sx/2,sy/2,1.]])@t.T
                boxes.append(dict(name=model.get('name','')+'/'+collision.get('name',''),polygon=points[:,:2].tolist(),z_bottom_m=z-sz/2,height_m=sz))
    if not boxes:raise ValueError('No supported wall boxes')
    return world.get('name','world'),boxes


def rasterize(boxes,resolution=.025,margin=.2):
    if not math.isfinite(resolution) or not .005<=resolution<=.2:raise ValueError('Invalid resolution')
    all_points=np.concatenate([b['polygon'] for b in boxes]);low=np.floor((all_points.min(axis=0)-margin)/resolution)*resolution
    high=np.ceil((all_points.max(axis=0)+margin)/resolution)*resolution
    width,height=np.rint((high-low)/resolution).astype(int)
    if width*height>4_000_000:raise ValueError('Map too large')
    xx,yy=np.meshgrid(low[0]+(np.arange(width)+.5)*resolution,low[1]+(np.arange(height)+.5)*resolution)
    occupied=np.zeros((height,width),bool)
    for box in boxes:
        polygon=np.array(box['polygon']);inside=np.ones_like(occupied)
        for a,b in zip(polygon,np.roll(polygon,-1,axis=0)):
            inside&=(b[0]-a[0])*(yy-a[1])-(b[1]-a[1])*(xx-a[0])>=-1e-10
        occupied|=inside
    # Padding exterior connects to all open regions. Only enclosed regions become free.
    free=(~occupied).astype(np.uint8);padded=np.pad(free,1,constant_values=1)
    cv2.floodFill(padded,None,(0,0),2)
    exterior=padded[1:-1,1:-1]==2
    image=np.full((height,width),205,np.uint8);image[~occupied&~exterior]=254;image[occupied]=0
    return np.flipud(image),low.tolist(),dict(x_min=float(all_points[:,0].min()),x_max=float(all_points[:,0].max()),y_min=float(all_points[:,1].min()),y_max=float(all_points[:,1].max()))


def convert(source,output,resolution=.025):
    world,boxes=boxes_from_world(source);image,origin,bounds=rasterize(boxes,resolution)
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    mapfile=output/(world+'.yaml');pgm=output/(world+'.pgm')
    cv2.imwrite(str(pgm),image)
    data=dict(image=pgm.name,mode='trinary',resolution=resolution,origin=origin+[0.],negate=0,occupied_thresh=.65,free_thresh=.196)
    mapfile.write_text(yaml.safe_dump(data,sort_keys=False))
    report=dict(source=str(Path(source).resolve()),source_type='SDF static collision boxes',world_name=world,wall_boxes=len(boxes),wall_bounds_m=bounds,map_resolution_m=resolution,map_size_pixels=[image.shape[1],image.shape[0]],occupied_cells=int((image==0).sum()),enclosed_free_cells=int((image==254).sum()),unknown_cells=int((image==205).sum()),world_coordinates_preserved=True,real_environment_and_scale_verified=False,plugins_executed=False)
    (output/'conversion-report.json').write_text(json.dumps(report,indent=2)+'\n')
    preview=cv2.cvtColor(cv2.resize(image,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST),cv2.COLOR_GRAY2BGR)
    # Display axes and scale in separate border so localization pixels stay untouched.
    preview=cv2.copyMakeBorder(preview,45,55,20,20,cv2.BORDER_CONSTANT,value=(245,245,245))
    cv2.putText(preview,world+' | 2.5cm/cell | source geometry, real scale unverified',(15,27),cv2.FONT_HERSHEY_SIMPLEX,.49,(30,30,30),1)
    length=int(1./resolution*4);y=preview.shape[0]-28
    cv2.line(preview,(25,y),(25+length,y),(30,30,30),3);cv2.putText(preview,'1 m',(30+length,y+5),cv2.FONT_HERSHEY_SIMPLEX,.55,(30,30,30),1)
    cv2.imwrite(str(output/'preview.png'),preview)
    print(json.dumps(report,indent=2));return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('output',type=Path);p.add_argument('--resolution',type=float,default=.025);a=p.parse_args();convert(a.source,a.output,a.resolution)
