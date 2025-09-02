from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('scan_fusion'),
            'config',
            'fusion_params.yaml'
        ]),
        description='fusion params config file path'
    )

    # 激光雷达融合节点
    laserscan_fusion_node = Node(
        package='scan_fusion',
        executable='scan_fusion_node',
        name='scan_fusion_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
        ],
        # remappings=[
        #     ('scan1', LaunchConfiguration('scan1_topic')),
        #     ('scan2', LaunchConfiguration('scan2_topic')),
        #     ('scan_fused', LaunchConfiguration('output_topic')),
        # ]
    )
    
    return LaunchDescription([
        config_file_arg,
        laserscan_fusion_node
    ])