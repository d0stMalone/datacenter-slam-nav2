"""Online SLAM with slam_toolbox, building a map while you drive.

Run it alongside the simulation:

    ros2 launch my_robot sim.launch.py rviz:=false     # terminal 1: Gazebo + robot
    ros2 launch my_robot slam.launch.py                # terminal 2: SLAM + mapping RViz
    ros2 run teleop_twist_keyboard teleop_twist_keyboard  # terminal 3: drive

Save the finished map (from the workspace root):

    ros2 run nav2_map_server map_saver_cli -f src/my_robot/maps/datahall

slam_toolbox subscribes to /scan and the odom->base_link TF, and publishes:
  * /map            nav_msgs/OccupancyGrid (the map image)
  * TF map->odom    the correction that cancels wheel-odometry drift
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_share, 'config', 'slam_params.yaml'),
            description='slam_toolbox parameter file'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),

        # "Async" = process the newest scan when the previous one is done, dropping scans if
        # it falls behind. Keeps the map->odom TF real-time, which is what you want while
        # driving. (The "sync" variant processes every scan, which is better for offline bags.)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('slam_params_file'),
                        {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'slam.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
