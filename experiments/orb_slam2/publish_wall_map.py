#!/usr/bin/env python3
"""Publish a saved local ORB-SLAM2 replay map for viewing in RViz."""
import argparse
import json
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray


def point(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


def make_marker(namespace, kind, rgb, size, frame):
    msg = Marker()
    msg.header.frame_id = frame
    msg.ns = namespace
    msg.type = kind
    msg.action = Marker.ADD
    msg.pose.orientation.w = 1.
    msg.scale.x = msg.scale.y = msg.scale.z = size
    msg.color.r, msg.color.g, msg.color.b = rgb
    msg.color.a = 1.
    return msg


def load(directory):
    data = json.loads((directory / 'wall-lines.json').read_text())
    frame = data['coordinate_frame']
    origin, basis = np.array(data['origin_in_orb_world_m']), np.array(data['axis_basis_in_orb_world'])
    trajectory = np.loadtxt(directory / 'trajectory.tum', ndmin=2)
    line = make_marker('camera_trajectory', Marker.LINE_LIST, (.14,.82,.87), .015, frame)
    xyz = (trajectory[:,1:4]-origin)@basis
    for i in np.flatnonzero(np.diff(trajectory[:,0]) <= .45):
        line.points.extend([point(*xyz[i]),point(*xyz[i+1])])
    room = make_marker('wall_edges', Marker.LINE_LIST, (1.,.66,.26), .018, frame)
    faces = make_marker('wall_surfaces', Marker.TRIANGLE_LIST, (.82,.84,.86), 1., frame)
    for wall in data['walls']:
        (x0,y0),(x1,y1) = wall['endpoints_xy_m']
        z0 = wall['floor_z_m']
        z1 = z0 + wall['height_m']
        a,b,c,d = point(x0,y0,z0),point(x1,y1,z0),point(x1,y1,z1),point(x0,y0,z1)
        room.points.extend([a,b,c,d])
        faces.points.extend([a,b,c,a,c,d])
    faces.color.a = .65
    return MarkerArray(markers=[line,room,faces])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args()
    message=load(args.directory)
    rclpy.init()
    node=Node('jetbot_orb_replay_map')
    publisher=node.create_publisher(MarkerArray,'/orb_slam2/replay_map',
                                   QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher.publish(message)
    node.get_logger().info('Fixed frame: orb_slam2_local_level, MarkerArray: /orb_slam2/replay_map')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
