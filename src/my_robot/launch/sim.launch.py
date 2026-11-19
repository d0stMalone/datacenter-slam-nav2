"""Bring up the simulation: Gazebo + a world + my_robot.

    ros2 launch my_robot sim.launch.py                       # data hall, Gazebo GUI + RViz
    ros2 launch my_robot sim.launch.py world:=warehouse      # the other built-in world
    ros2 launch my_robot sim.launch.py gui:=false            # headless physics (faster)
    ros2 launch my_robot sim.launch.py x:=0.0 y:=-5.0 yaw:=1.57

Starts:
  1. gzserver (physics + sensors) with the world, and optionally gzclient (the 3D window)
  2. robot_state_publisher (rsp.launch.py) -> publishes /robot_description and URDF TFs
  3. spawn_entity.py -> reads /robot_description and inserts the robot into Gazebo
  4. RViz (optional) with a pre-made config
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_SHARE = get_package_share_directory('my_robot')

# Built-in worlds and a known-free spawn pose (x, y, yaw) in each.
WORLD_SPAWNS = {
    'datahall': ('-6.0', '-5.2', '0.0'),
    'warehouse': ('-5.8', '0.0', '0.0'),
    'datacenter': ('-4.95', '-2.75', '0.0'),   # hall W cross aisle, east end
}


def resolve_world_and_spawn(context):
    """Turn the `world` argument into a file path and fill in any spawn values not given."""
    world = LaunchConfiguration('world').perform(context)
    if world in WORLD_SPAWNS:
        world_path = os.path.join(PKG_SHARE, 'worlds', f'{world}.world')
        default_spawn = WORLD_SPAWNS[world]
    else:  # a path to some other .world file
        world_path, default_spawn = world, ('0.0', '0.0', '0.0')

    x, y, yaw = (LaunchConfiguration(k).perform(context) or d
                 for k, d in zip(('x', 'y', 'yaw'), default_spawn))
    return world_path, x, y, yaw


def launch_setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path, x, y, yaw = resolve_world_and_spawn(context)
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(PKG_SHARE, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_my_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            # z = base_link height = wheel radius, +1 cm so the robot drops onto the floor.
            '-x', x, '-y', y, '-z', '0.06', '-Y', yaw,
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [gzserver, gzclient, rsp, spawn, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world', default_value='datahall',
            description=f'Built-in world name {list(WORLD_SPAWNS)} or a path to a .world file'),
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(PKG_SHARE, 'rviz', 'sim.rviz')),
        # Spawn pose; empty = use the chosen world's default from WORLD_SPAWNS.
        DeclareLaunchArgument('x', default_value=''),
        DeclareLaunchArgument('y', default_value=''),
        DeclareLaunchArgument('yaw', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
