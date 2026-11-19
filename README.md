# Data-centre SLAM & autonomous navigation robot (ROS 2 Humble)

A differential-drive robot that maps data-centre environments with **slam_toolbox** and
navigates them autonomously with **Nav2 + AMCL**, simulated in **Gazebo Classic**.

> 🚧 Work in progress. Phases 1–4 are done; autonomy (frontier exploration, waypoint patrol)
> and a full README are coming. Detailed technical write-ups for every phase are in
> [`docs/`](docs/README.md).

## Highlights so far
- Custom robot (Xacro URDF): diff drive, 2D lidar, camera, ground-truth pose for evaluation
- Two generated worlds: a 20 × 14 m data hall and a **162 × 38 m full data-centre building**
  modelled on a real floor plan
- SLAM map of the data hall: **99.9 % precision** against ground truth
- Nav2 tuned for 1.2 m rack aisles: centred straight paths (Theta\*), ≤ 10 cm localisation error
- Keep-out / avoid zones (Nav2 costmap filters) generated from a simple YAML file

## Quick start
```bash
# Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
git clone <this repo> .
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install && source install/setup.bash

ros2 launch my_robot sim.launch.py rviz:=false      # terminal 1: Gazebo
ros2 launch my_robot nav.launch.py                  # terminal 2: Nav2 + RViz, click "Nav2 Goal"
```

## Documentation
| Phase | Doc |
|---|---|
| 1. Environment | [docs/01-environment.md](docs/01-environment.md) |
| 2. Robot & world | [docs/02-robot-and-world.md](docs/02-robot-and-world.md) |
| 3. SLAM | [docs/03-slam.md](docs/03-slam.md) |
| 4. Navigation | [docs/04-navigation.md](docs/04-navigation.md) |

## License
MIT
