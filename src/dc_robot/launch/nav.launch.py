"""Autonomous navigation: Nav2 + AMCL on a saved map, or Nav2 + SLAM while mapping (+ RViz).

Run the simulation first, then this:

    ros2 launch dc_robot sim.launch.py rviz:=false
    ros2 launch dc_robot nav.launch.py

For another world, pass the same name to both (it selects maps/<world>.yaml and
maps/<world>_keepout.yaml):

    ros2 launch dc_robot sim.launch.py world:=datacenter rviz:=false
    ros2 launch dc_robot nav.launch.py world:=datacenter

Navigate while mapping an unknown place (slam_toolbox instead of map_server + AMCL; Nav2
plans on the growing SLAM map and dodges obstacles with the live lidar):

    ros2 launch dc_robot nav.launch.py slam:=true keepout:=false

In RViz: the robot starts localised (AMCL initial pose = default spawn). Click
"Nav2 Goal" and drag on the map to send the robot somewhere.

What gets started (via nav2_bringup/bringup_launch.py):
    map_server          serves maps/<world>.yaml on /map
    amcl                localises the robot on that map, publishes TF map -> odom
    planner_server      global path (Theta*, straight any-angle segments) on the global costmap
    controller_server   follows the path (Regulated Pure Pursuit) -> /cmd_vel_nav
    velocity_smoother   limits speed/accel: /cmd_vel_nav -> /cmd_vel
    behavior_server     recovery behaviours (spin, back up, wait)
    bt_navigator        behaviour tree tying it together; serves the NavigateToPose action
    waypoint_follower, smoother_server, lifecycle managers
Also: depth_cloud_relay   /camera/points -> /camera/points_xyz (thin cloud for the voxel layer)

Keep-out zones (added here, outside nav2_bringup), if keepout:=true (default):
    filter_mask_server          map_server publishing maps/<world>_keepout on /keepout_filter_mask
    costmap_filter_info_server  tells the costmaps' KeepoutFilter where the mask is
    lifecycle_manager_costmap_filters  brings those two up
Edit zones in config/keepout_zones*.yaml, then run scripts/generate_keepout_mask.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('dc_robot')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')
    keepout = LaunchConfiguration('keepout')
    keepout_mask = LaunchConfiguration('keepout_mask')
    world = LaunchConfiguration('world')
    maps_dir = os.path.join(pkg, 'maps')
    slam = LaunchConfiguration('slam')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Gazebo /clock'),
        DeclareLaunchArgument('world', default_value='datahall',
                              description='World name: picks the default map and keep-out mask'),
        DeclareLaunchArgument('map',
                              default_value=PathJoinSubstitution([maps_dir, [world, '.yaml']]),
                              description='Full path to the map yaml to navigate on'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
                              description='Nav2 parameters'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Bring the Nav2 lifecycle nodes up automatically'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Map while navigating (slam_toolbox) instead of AMCL'),
        DeclareLaunchArgument('keepout', default_value='true',
                              description='Enforce the keep-out / avoid zones'),
        DeclareLaunchArgument('keepout_mask',
                              default_value=PathJoinSubstitution(
                                  [maps_dir, [world, '_keepout.yaml']]),
                              description='Keep-out mask yaml (from generate_keepout_mask.py)'),
    ]

    # nav2_bringup's top-level launch = localization (map_server + amcl) + navigation servers.
    # It rewrites use_sim_time and the map path into every node's parameters for us.
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': 'True',   # all servers in one process: faster intra-process comms
            # bringup_launch.py has its own 'slam' argument, evaluated as Python ('not False').
            # Launch arguments are global, so without this our slam:=false would leak into it
            # as 'not false' -> NameError.
            'slam': 'False',
        }.items(),
        condition=UnlessCondition(slam),
    )

    # SLAM mode: our slam_toolbox setup (publishes /map and map -> odom) + Nav2's navigation
    # servers only (no map_server / AMCL; those would fight SLAM over the same TF).
    slam_mode = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'slam.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'rviz': 'false'}.items(),
            condition=IfCondition(slam)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
            launch_arguments={'use_sim_time': use_sim_time, 'params_file': params_file,
                              'autostart': autostart}.items(),
            condition=IfCondition(slam)),
    ]

    # Keep-out zones. The costmaps' KeepoutFilter (configured in nav2_params.yaml) waits for
    # /costmap_filter_info; without these nodes (keepout:=false) it simply does nothing.
    keepout_nodes = [
        Node(
            package='nav2_map_server', executable='map_server', name='filter_mask_server',
            parameters=[params_file, {'use_sim_time': use_sim_time,
                                      'yaml_filename': keepout_mask}],
            condition=IfCondition(keepout), output='screen'),
        Node(
            package='nav2_map_server', executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            condition=IfCondition(keepout), output='screen'),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                         'node_names': ['filter_mask_server', 'costmap_filter_info_server']}],
            condition=IfCondition(keepout), output='screen'),
    ]

    # Thins the depth camera's cloud for the costmaps' voxel layer (see depth_cloud_relay.py).
    depth_relay = Node(
        package='dc_robot', executable='depth_cloud_relay', name='depth_cloud_relay',
        parameters=[{'use_sim_time': use_sim_time}], output='screen')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription(args + [bringup] + slam_mode + keepout_nodes
                             + [depth_relay, rviz_node])
