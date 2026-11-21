# Data-centre SLAM & autonomous navigation robot (ROS 2 Humble)

A differential-drive robot that maps data-centre environments with **slam_toolbox** and
navigates them autonomously with **Nav2 + AMCL**, simulated in **Gazebo Classic**.

> 🚧 Work in progress. Phases 1–4 (and 4b, the whole data centre) are done; autonomy (frontier exploration, waypoint patrol)
> and a full README are coming. Detailed technical write-ups for every phase are in
> [`docs/`](docs/README.md).

## Highlights so far
- Custom robot (Xacro URDF): diff drive, 0.25° 2D lidar, RGB-D depth camera, ground-truth pose for evaluation
- Two generated worlds: a 20 × 14 m data hall and a **162 × 38 m full data-centre building**
  modelled on a real floor plan
- SLAM map of the data hall: **99.9 % precision** against ground truth
- Nav2 tuned for 1.2 m rack aisles: centred straight paths (Theta\*), ≤ 10 cm localisation error
- Keep-out / avoid zones (Nav2 costmap filters) generated from a simple YAML file
- Floor-plan (CAD-style) map generated from the world; AMCL tuned for identical, repeating aisles
- People, ladders and carts in the aisles that are **not on the map**: a depth camera + 3D voxel
  costmap catches what the 2D lidar misses (a cart's bottom shelf); 7/7 goals, detours around
  a blocked aisle

## Quick start
```bash
# Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone <this repo> .
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash

ros2 launch dc_robot sim.launch.py rviz:=false      # terminal 1: Gazebo
ros2 launch dc_robot nav.launch.py                  # terminal 2: Nav2 + RViz, click "Nav2 Goal"
```

Whole data-centre building (add `_busy` for people, ladders and carts in the aisles):
```bash
ros2 launch dc_robot sim.launch.py world:=datacenter_busy rviz:=false
ros2 launch dc_robot nav.launch.py world:=datacenter            # floor-plan map
ros2 launch dc_robot nav.launch.py world:=datacenter map:=$(ros2 pkg prefix dc_robot)/share/dc_robot/maps/datacenter_slam.yaml   # or the SLAM map
```

## Documentation
| Phase | Doc |
|---|---|
| 1. Environment | [docs/01-environment.md](docs/01-environment.md) |
| 2. Robot & world | [docs/02-robot-and-world.md](docs/02-robot-and-world.md) |
| 3. SLAM | [docs/03-slam.md](docs/03-slam.md) |
| 4. Navigation | [docs/04-navigation.md](docs/04-navigation.md) |
| 4b. Whole data centre, clutter, depth camera | [docs/04b-datacenter.md](docs/04b-datacenter.md) |

## License
MIT
