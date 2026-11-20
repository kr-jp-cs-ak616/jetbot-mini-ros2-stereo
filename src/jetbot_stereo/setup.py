from glob import glob
from setuptools import setup

setup(name='jetbot_stereo', version='0.1.0', packages=['jetbot_stereo'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/jetbot_stereo']),
                  ('share/jetbot_stereo', ['package.xml']),
                  ('share/jetbot_stereo/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['csi_stereo = jetbot_stereo.capture:main',
                                       'socket_camera = jetbot_stereo.socket_camera:main',
                                       'decode_stereo = jetbot_stereo.decode_stereo:main',
                                       'boundary_walls = jetbot_stereo.boundary_node:main',
                                       'wall_map_view = jetbot_stereo.map_view:main']})
