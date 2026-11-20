from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from jetbot_stereo.calibration import load_calibration, validate_pair


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    mapping = value('mapping').lower() == 'true'
    for name in ('topic_queue_size', 'sync_queue_size', 'max_disparity'):
        if int(value(name)) < 1:
            raise ValueError(name + ' must be positive')
    directory = value('calibration_dir')
    width, height = int(value('width')), int(value('height'))
    if mapping:
        if not directory:
            raise ValueError('Mapping requires calibration_dir with joint stereo left.yaml/right.yaml')
        pair = [load_calibration(Path(directory) / (s + '.yaml'), width, height)
                for s in ('left', 'right')]
        validate_pair(*pair)
    nodes = []
    if value('start_capture').lower() == 'true':
        nodes.append(Node(package='jetbot_stereo', executable='csi_stereo', output='screen',
                          parameters=[dict(width=width, height=height, fps=int(value('fps')),
                                           left_sensor_id=int(value('left_sensor_id')),
                                           right_sensor_id=int(value('right_sensor_id')),
                                           sensor_mode=int(value('sensor_mode')),
                                           flip_method=int(value('flip_method')),
                                           calibration_dir=directory)]))
    if not mapping:
        return nodes
    if value('start_rectification').lower() == 'true':
        nodes.append(ComposableNodeContainer(
            name='rectify_container', namespace='', package='rclcpp_components',
            executable='component_container', output='screen',
            composable_node_descriptions=[ComposableNode(
                package='image_proc', plugin='image_proc::RectifyNode', name='rectify',
                namespace=f'/stereo/{side}',
                remappings=[('image', 'image_raw'), ('image_rect', 'image_rect')])
                for side in ('left', 'right')]))
    # Camera-only base: x forward, y left, z up. No invented robot mounting offset.
    nodes.append(Node(package='tf2_ros', executable='static_transform_publisher',
                      name='left_optical_tf', arguments=[
                          '--x', '0', '--y', '0', '--z', '0',
                          '--roll', '-1.5707963267948966', '--pitch', '0',
                          '--yaw', '-1.5707963267948966',
                          '--frame-id', 'stereo_camera_link',
                          '--child-frame-id', 'stereo_left_optical_frame']))
    database = Path(value('database_path')).expanduser().resolve()
    database.parent.mkdir(parents=True, exist_ok=True)
    nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('rtabmap_launch')) / 'launch/rtabmap.launch.py')),
        launch_arguments={
            'stereo': 'true', 'depth': 'false', 'visual_odometry': 'true',
            'frame_id': 'stereo_camera_link', 'vo_frame_id': 'stereo_odom',
            'odom_topic': '/stereo/odom', 'map_topic': '/map',
            'left_image_topic': '/stereo/left/image_rect',
            'right_image_topic': '/stereo/right/image_rect',
            'left_camera_info_topic': '/stereo/left/camera_info',
            'right_camera_info_topic': '/stereo/right/camera_info',
            'approx_sync': 'true', 'approx_sync_max_interval': value('sync_tolerance'),
            'topic_queue_size': value('topic_queue_size'),
            'sync_queue_size': value('sync_queue_size'),
            'args': '--Stereo/MaxDisparity ' + value('max_disparity'),
            'qos': '2', 'rtabmap_viz': value('viz'), 'rviz': 'false',
            'database_path': str(database),
        }.items()))
    return nodes


def generate_launch_description():
    args = dict(mapping='false', start_capture='true', start_rectification='true', calibration_dir='',
                width='640', height='480', fps='15', left_sensor_id='0',
                right_sensor_id='1', sensor_mode='-1', flip_method='0',
                sync_tolerance='0.02', topic_queue_size='5', sync_queue_size='5',
                max_disparity='256', viz='false',
                database_path='/data/maps/jetbot.db')
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args.items()]
                             + [OpaqueFunction(function=setup)])
