import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('test_pkg')
    
    # 强制使用仿真时间
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',  # 设为true
        description='Use simulation (Gazebo) clock'
    )
    
    # RViz配置
    rviz_config_file = os.path.join(pkg_share, 'launch/check_joint.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        rviz_node
    ])