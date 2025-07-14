from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),  
                    'launch',
                    'turtlebot3_autorace_2020.launch.py'
                ])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_autorace_camera'),
                    'launch',
                    'extrinsic_camera_calibration.launch.py'
                ])
            ),
            launch_arguments={
                'calibration_mode': 'True'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_autorace_detect'),
                    'launch',
                    'detect_traffic_light.launch.py'
                ])
            ),
            launch_arguments={
                'calibration_mode': 'True'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_autorace_camera'),
                    'launch',
                    'intrinsic_camera_calibration.launch.py'
                ])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_autorace_detect'),
                    'launch',
                    'detect_lane.launch.py'
                ])
            ),
            launch_arguments={
                'calibration_mode': 'True'
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_autorace_mission'),
                    'launch',
                    'control_lane.launch.py'
                ])
            )
        )
    ])