from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 启动Gazebo空世界
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'verbose': 'false',
                'world': PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'worlds',
                    'empty.world'
                ]),
                'use_sim_time': 'true'  # 确保Gazebo使用仿真时间
            }.items()
        ),

        # 发布机器人描述（唯一robot_state_publisher）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('test_pkg'),
                        'xacro',
                        'robot.xacro'
                    ])
                ]),
                'use_sim_time': True  # 同步Gazebo时间
            }]
        ),

        # 生成URDF模型到Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            output='screen',
            arguments=[
                '-entity', 'go2',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0', 
                '-z', '0.5'
            ]
        ),

        # 控制器校准
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                ' topic pub ',
                '/calibrated ',
                'std_msgs/msg/Bool ',
                '"{data: true}" ',
                '--once'
            ]],
            shell=True
        )
    ])