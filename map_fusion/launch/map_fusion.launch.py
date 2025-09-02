from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # 在仿真中设置为true
        description='Use simulation time'
    )

    # 声明launch参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('map_fusion'),
            'config',
            'map_fusion_params.yaml'
        ]),
        description='Map fusion parameters config file path'
    )

    # 地图融合节点
    map_fusion_node = Node(
        package='map_fusion',
        executable='map_fusion_node',
        name='map_fusion_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('config_file')
        ],
        remappings=[
            ('/base_map', "/lidar_map"),
            ('/affixion_map', "/depth_map"),
            ('/fused_map', "/fused_map"),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        
        map_fusion_node
    ])