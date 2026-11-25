"""Locate the stereo camera in a fixed occupancy map without a SLAM writer."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as Config
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('calibration_dir'),
        DeclareLaunchArgument('camera_height_m', default_value='0.099'),
        DeclareLaunchArgument('camera_pitch_deg', default_value='0.0'),
        DeclareLaunchArgument('partition_height_m', default_value='0.0'),
        DeclareLaunchArgument('partition_pitch_tolerance_deg', default_value='0.0'),
        DeclareLaunchArgument('min_panel_luma', default_value='75.0'),
        DeclareLaunchArgument('show_view', default_value='false'),
        DeclareLaunchArgument('rviz_config', default_value=''),
        Node(package='jetbot_stereo', executable='boundary_walls', output='screen',
             parameters=[{
                 'localization_only': True,
                 'guard_reference_odometry': True,
                 'calibration_dir': Config('calibration_dir'),
                 'camera_height_m': ParameterValue(Config('camera_height_m'), value_type=float),
                 'camera_pitch_deg': ParameterValue(Config('camera_pitch_deg'), value_type=float),
                 'partition_height_m': ParameterValue(Config('partition_height_m'), value_type=float),
                 'partition_pitch_tolerance_deg': ParameterValue(
                     Config('partition_pitch_tolerance_deg'), value_type=float),
                 'min_panel_luma': ParameterValue(Config('min_panel_luma'), value_type=float),
             }]),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=[
            '--roll', '-1.5707963267948966', '--pitch', '0', '--yaw', '-1.5707963267948966',
            '--frame-id', 'stereo_camera_link', '--child-frame-id', 'stereo_left_optical_frame',
        ]),
        Node(package='rtabmap_odom', executable='stereo_odometry', name='reference_odometry',
             output='screen', parameters=[{
                 'frame_id': 'stereo_camera_link', 'odom_frame_id': 'partition_odom',
                 'publish_tf': False,
                 'approx_sync': True, 'approx_sync_max_interval': .025,
                 'topic_queue_size': 5, 'sync_queue_size': 5,
                 'qos': 2, 'qos_camera_info': 2, 'Stereo/MaxDisparity': '256',
             }], remappings=[
                 ('left/image_rect', '/stereo/left/image_rect'),
                 ('right/image_rect', '/stereo/right/image_rect'),
                 ('left/camera_info', '/stereo/left/camera_info'),
                 ('right/camera_info', '/stereo/right/camera_info'),
                 ('odom', '/partition/odom'),
             ]),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': Config('map_yaml')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[{
                 'base_frame_id': 'stereo_camera_link', 'odom_frame_id': 'partition_odom',
                 'scan_topic': '/partition/wall_scan', 'first_map_only': True,
                 'laser_model_type': 'likelihood_field', 'laser_max_range': -1.0,
                 'max_beams': 91, 'sigma_hit': .15, 'update_min_a': .05, 'update_min_d': .04,
                 'resample_interval': 1,
             }]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}]),
        Node(package='rviz2', executable='rviz2', condition=IfCondition(Config('show_view')),
             arguments=['-d', Config('rviz_config')]),
    ])
