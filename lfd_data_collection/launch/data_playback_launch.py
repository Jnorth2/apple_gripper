from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    
    # Launch the ur5e moveit2 launch file 
    # (also launches rviz and the moveit_service.cpp moveit node)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gripper'),
                'launch',
                'ur_moveit_playback_launch.py'
            ])
        ]),
    ))
    
    # Launch the node to simulate the gripper in moveit
    ld.add_action(Node(
        package='lfd_data_collection',
        executable='moveit_playback',
    ))

    ld.add_action(DeclareLaunchArgument('arg1', default_value='optitrack/Take 2024-08-16 12.02.44 AM.csv', description='the optitrack file relative to the data folder'))
    ld.add_action(DeclareLaunchArgument('arg2', default_value=str('bags/20240816_8'), description='the ros bag file relative to the data folder'))

    # Launch the node that reads optitrack data
    ld.add_action(Node(
        package='lfd_data_collection',
        executable='optitrack.py',
        parameters=[{
            'arg1': LaunchConfiguration('arg1'),
            'arg2': LaunchConfiguration('arg2')
        }]
    ))
    
    # Launch the apple attach node
    ld.add_action(Node(
    	package='lfd_data_collection',
    	executable='apple_attach.py',
    ))
    
    
    return ld
