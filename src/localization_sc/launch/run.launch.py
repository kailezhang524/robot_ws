import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
def generate_launch_description():
    config_file_path = os.path.join(get_package_share_directory('localization_sc'), 'config', 'config.yaml')
    mapping_file_path =os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
    livox_file_path =os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360s_launch.py')
    # Launch arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/Odometry')
    lidar_topic_arg = DeclareLaunchArgument('lidar_topic', default_value='/cloud_registered')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    mapping= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_file_path),
        launch_arguments={"use_rviz": "true"}.items()
    )
    # fast_lio_sam_dir = get_package_share_directory('fast_lio_sam_sc_qn')
    livox_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_file_path)
    )
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time_cmd,
        livox_lidar,
        mapping,
        rviz_arg,
        odom_topic_arg,
        lidar_topic_arg,

        # # RViz node (optional)
        # Node(
        #     condition=IfCondition(LaunchConfiguration('rviz')),
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz_sam_localization',
        #     arguments=['-d', PathJoinSubstitution([fast_lio_loc_dir, 'config', 'localization_rviz.rviz'])],
        #     output='screen'
        # ),

        # Fast LIO Localization node
        Node(
            package='localization_sc',
            executable='localization_sc_node',
            name='localization_sc_node',
            output='screen',
            parameters=[config_file_path,{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/Odometry', LaunchConfiguration('odom_topic')),
                ('/cloud_registered', LaunchConfiguration('lidar_topic'))
            ]
        ),
    ])