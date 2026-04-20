import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
     # Find path
    rviz_config_file = os.path.join(get_package_share_directory('pgo_sc'), 'config', 'sam_rviz.rviz')
    config_file_path = os.path.join(get_package_share_directory('pgo_sc'), 'config', 'config.yaml')
    mapping_file_path =os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
    livox_file_path =os.path.join(get_package_share_directory('livox_ros_driver2'), 'launch_ROS2', 'msg_MID360s_launch.py')

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Whether to launch RViz2"
    )

    # odom_topic: 里程计 topic remap
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='/Odometry')
    # lidar_topic: 点云 topic remap
    lidar_topic_arg = DeclareLaunchArgument('lidar_topic', default_value='/cloud_registered')
    
    mapping= IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_file_path),
        launch_arguments={"use_rviz": "true"}.items()
    )
    livox_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(livox_file_path)
    )
    return LaunchDescription([
        use_rviz_arg,
        odom_topic_arg,
        lidar_topic_arg,
        mapping,
        # livox_lidar,
        Node(       
            package='pgo_sc',
            executable='pgo_sc_node',
            name='pgo_sc_node',
            parameters=[config_file_path],
            remappings=[
                ('/Odometry', LaunchConfiguration('odom_topic')),# <remap from="/Odometry" to="...">
                ('/cloud_registered', LaunchConfiguration('lidar_topic'))
            ],
            output='screen'
        ),
        # Node(
        #     condition=IfCondition(LaunchConfiguration("use_rviz")),
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     arguments=["-d", rviz_config_file],
        #     output="screen"
        # ),

    ])