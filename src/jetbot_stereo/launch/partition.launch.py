"""Use preprocessed transient odometry; persist sparse partition wall cells only."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    c=LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('calibration_dir'),DeclareLaunchArgument('map_file'),
        DeclareLaunchArgument('camera_height_m',default_value='0.099'),
        DeclareLaunchArgument('camera_pitch_deg',default_value='0.0'),
        DeclareLaunchArgument('tracking_masked_only',default_value='false'),
        DeclareLaunchArgument('show_view',default_value='false'),DeclareLaunchArgument('rviz_config',default_value=''),
        Node(package='jetbot_stereo',executable='wall_map_view',output='screen',
             condition=IfCondition(c('show_view')),parameters=[{'map_file':c('map_file')}]),
        Node(package='rviz2',executable='rviz2',name='partition_wall_view',
             condition=IfCondition(c('show_view')),arguments=['-d',c('rviz_config')]),
        Node(package='jetbot_stereo',executable='boundary_walls',output='screen',parameters=[{
            'calibration_dir':c('calibration_dir'),'map_file':c('map_file'),
            'camera_height_m':ParameterValue(c('camera_height_m'),value_type=float),
            'camera_pitch_deg':ParameterValue(c('camera_pitch_deg'),value_type=float),
            'tracking_masked_only':ParameterValue(c('tracking_masked_only'),value_type=bool)}]),
        Node(package='tf2_ros',executable='static_transform_publisher',name='partition_optical_tf',arguments=[
            '--roll','-1.5707963267948966','--pitch','0','--yaw','-1.5707963267948966',
            '--frame-id','stereo_camera_link','--child-frame-id','stereo_left_optical_frame']),
        Node(package='rtabmap_odom',executable='stereo_odometry',name='partition_odometry',output='screen',parameters=[{
            'frame_id':'stereo_camera_link','odom_frame_id':'partition_odom',
            'approx_sync':True,'approx_sync_max_interval':.02,'topic_queue_size':5,'sync_queue_size':5,
            'qos':2,'qos_camera_info':2,'Stereo/MaxDisparity':'256','OdomF2M/MaxSize':'1000','publish_tf':True}],remappings=[
            ('left/image_rect','/partition/tracking/left/image_rect'),('right/image_rect','/partition/tracking/right/image_rect'),
            ('left/camera_info','/partition/left/camera_info'),('right/camera_info','/partition/right/camera_info'),
            ('odom','/partition/odom')])])
