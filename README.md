# RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry
[**YouTube**](https://youtu.be/3-xLRRT25ys) | **[arXiv](https://arxiv.org/abs/2504.11580)** | **[Website](https://asig-x.github.io/resple_web/)** | **[IEEE RA-L](https://doi.org/10.1109/LRA.2025.3604758)** | **[Demonstrator](https://asig-x.github.io/resple_demonstrator/)**

--> [Branch for benchmarking](https://github.com/ASIG-X/RESPLE/tree/feature/benchmark)
### News
* 2026-06: A new feature is now available to save maps as `.pcd` files. The `RESPLE` node exposes a `save_map` service to save the ikd-tree map on demand. A new `MapSaving` node is also available to accumulate and save the dense global map published by the `Mapping` node. Please check out the branch [here](https://github.com/ASIG-X/RESPLE/tree/feature/save_map).
* 2026-01: A new feature is now available to save estimated trajectories specifically for benchmarking. Please check out the branch [here](https://github.com/ASIG-X/RESPLE/tree/feature/benchmark). An example using the HelmDyn dataset can be found [here](https://github.com/ASIG-X/RESPLE/blob/feature/benchmark/resple/config/config_helmdyn01.yaml). The estimated trajectory will be generated through spline interpolation at ground-truth timestamps in a `.txt` file following the [TUM format](https://github.com/MichaelGrupp/evo/wiki/Formats).
* 2025-12: The design of a handheld demostrator for RESPLE is now publicly available. Check out our web page [here](https://asig-x.github.io/resple_demonstrator/).
* 2025-12: Additional evaluation results of RESPLE-LIO and corresponding parameter sets on the [Newer College](https://ori-drs.github.io/newer-college-dataset/) dataset (including its extension) and the [MCD](https://mcdviral.github.io/) dataset are now available on our [web page](https://asig-x.github.io/resple_web/add_evaluation.html). Instructions for testing are given below.
* 2025-09: The [TudoRun](https://asig-x.github.io/resple_web/datasets.html) dataset is released as a supplementary dataset.


This is the offcial repository for RESPLE, the first B-spline-based recursive state estimation framework for estimating 6-DoF dynamic motions. Using RESPLE as the estimation backbone, we developed a unified suite of direct LiDAR-based odometry systems, including:
* LiDAR-only odometry (LO)
* LiDAR-inertial odometry (LIO)
* Multi-LiDAR odometry (MLO)
* Multi-LiDAR-inertial Odometry (MLIO)

These four variants have been tested in real-world datasets and our own experiments, covering aerial, wheeled, legged, and wearable platforms operating in indoor, urban, wild environments with diverse LiDAR types. We look forward to your comments and feedback! 

### BibTex Citation
```
@ARTICLE{cao2025resple,
  author={Cao, Ziyu and Talbot, William and Li, Kailai},
  title={RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry}, 
  journal={IEEE Robotics and Automation Letters},
  volume={10},
  number={10},
  pages={10666-10673},
  year={2025}
}
``` 
### Dependencies
Tested with [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) on Ubuntu 24.04
```
sudo apt install libomp-dev libpcl-dev libeigen3-dev
sudo apt install ros-jazzy-pcl*
# Optional: sudo apt install ros-jazzy-rosbag2-storage-mcap (for playing .mcap file if testing GrandTour dataset)
```


### Compilation
```
cd ~/ros2_ws/src
git clone --recursive https://github.com/ASIG-X/RESPLE.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select estimate_msgs livox_ros_driver livox_interfaces livox_ros_driver2 resple
```

## Docker Build

A ready-to-use ROS2 Jazzy development image is provided under `docker/`:

```bash
cd ~/path/to/src
git clone --recursive https://github.com/ASIG-X/RESPLE.git
cd RESPLE/docker
./build.sh
```

This builds the `resple:jazzy` image (pass `./build.sh --no-cache` to force a clean rebuild).

## Own experimental datasets ([LINK to SURFdrive](https://surfdrive.surf.nl/files/index.php/s/lfXfApqVXTLIS9l)) 
Password: RESPLE2025

<!-- ![image](doc/real_experiment2.gif) -->
<!-- [![Watch the video](doc/real_exp_2.png)](https://youtu.be/2OvjGnxszf8) -->
<div align="left">
<img src="doc/hemdyn_clip.gif" width=49.6% />
<img src="doc/Rcampus_clip.gif" width = 49.6% >
</div>
<br>

**HelmDyn (Helm Dynamic) dataset**
* 1 Livox Mid360 mounted on a helmet as a mobile platform
* 10 sequences recorded with very dynamic motions combining walking, running, jumping, and in-hand waving within a cubic space   
* Ground truth trajectory recorded using a high-precision (submillimeter), low-latency motion capture system (Qualisys) involving 20 cameras

**R-Campus dataset**
* 1 Livox Avia mounted on a bipedal wheeled robot (Direct Drive DIABLO)
* 1 sequence in walking speed recorded in a large-scale campus environment
* Trajectory starts and ends at the same location point. 

**TudoRun (Tudor Run) dataset**
* 1 Livox Mid360 mounted on a Unitree Go2 quadruped robot
* 8 indoor sequences with dynamic motions: 3 fully captured in a test field with an 8-camera motion capture system, and 5 starting and ending in the test field but extending into a larger hall without motion capture
* Ground truth trajectory recorded only within this test field using the motion capture system (Qualisys) with passive markers

**Please refer to our [dataset website](https://asig-x.github.io/resple_web/datasets.html) for more information.**
## Usage
Every dataset/sensor setup is launched through the same generic launch file:
```
source install/setup.bash
ros2 launch resple resple.launch.py config:=<name>
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```
Optional launch arguments:
* `rviz:=false` — skip launching RViz (default `true`)
* `publish_static_tf:=true` — also publish an identity `map`->`my_frame` static transform, for configs with no map-frame source of their own (default `false`)

Available `<name>` values, each backed by `resple/config/config_<name>.yaml`:

| `config:=` | Sensors | Mode |
|---|---|---|
| `hap360` | Livox HAP360 + IMU | LiDAR-Inertial |
| `hap360_lidaronly` | Livox HAP360 | LiDAR-only |
| `mid360` | Livox Mid360 (PointCloud2) + IMU | LiDAR-Inertial |
| `mid360_hesai_lidaronly` | Livox Mid360 + Hesai | LiDAR-only, multi-LiDAR |
| `ouster` | Ouster + IMU | LiDAR-Inertial |
| `ouster_lidaronly` | Ouster | LiDAR-only |

Each `config_<name>.yaml` is commented inline and grouped into `imu:`, `lidar:`, `spline:`, `mapping:` and `wheel_odometry:` sections. Copy the closest one as a starting point for your own sensor setup, and toggle `if_lidar_only` to switch between LiDAR-only and LiDAR-Inertial for a given sensor.

## Wheel Odometry
RESPLE can optionally fuse wheel/velocity odometry as a tightly-coupled IEKF factor, evaluated analytically against the B-spline's continuous linear and angular velocity, with adaptive covariance inflation to reject wheel slip. It is disabled by default; enable it per-config under `wheel_odometry:` (see `config_hap360.yaml` or `config_mid360.yaml` for the full commented template):

```yaml
wheel_odometry:
  enable: true
  topic_name: "/wheel_odom"
  topic_type: "nav_msgs/msg/Odometry"    # or "geometry_msgs/msg/TwistStamped"
  use_only_vx: true                      # restrict the update to the forward-velocity axis

  # Extrinsic from IMU/base (B) to the wheel-odometry frame (O), same convention as LiDAR's q_lb/t_lb
  q_wb: [1.0, 0.0, 0.0, 0.0]
  t_wb: [0.0, 0.0, 0.0]

  std_vx: 0.05
  std_vy: 0.02
  std_vz: 0.02

  max_allowed_residual_vx: 0.5           # residual (m/s) above which the update is treated as wheel slip
  adaptive_covariance_multiplier: 100.0  # covariance inflation applied when slip is detected
```

## Docker Usage

With the image built (see Docker Build above), run the algorithm in a docker container via `docker compose`.

Allow the docker user to generate graphics:
```bash
xhost +local:docker
```

From `docker/`, start the container in the background (the repo root is mounted at `~/workspace/src/RESPLE`, and `build/`, `install/`, `log/` are mounted for development so they persist outside the container):
```bash
cd docker
docker compose up -d
docker exec -it resple_jazzy bash
```

Inside the container, build the workspace with the `cb` alias (set up in `~/.bashrc`):
```bash
cb   # colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash
```

Launch, replacing `<name>` with one of the configs from Usage above:
```bash
ros2 launch resple resple.launch.py config:=<name>
```

Open a second terminal attached to the same running container and play a bag:
```bash
docker exec -it resple_jazzy bash
ros2 bag play /path/to/bag/
```

Manage the container from `docker/`:
* `docker compose stop` — stop it
* `docker compose up -d` — start it again
* `docker compose down` — remove it

## Contributors
Ziyu Cao (Email: ziyu.cao@liu.se)

William Talbot (Email: wtalbot@ethz.ch)

Kailai Li (Email: kailai.li@rug.nl)

Pau Reverté (Email: pau.reverte@eurecat.org)

## Credits
Thanks for [SFUISE](https://github.com/ASIG-X/SFUISE), [ikd-Tree](https://github.com/hku-mars/ikd-Tree), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [Livox-SDK](https://github.com/Livox-SDK), and [basalt](https://gitlab.com/VladyslavUsenko/basalt).

## License
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.
