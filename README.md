# RESPLE: Recursive Spline Estimation for LiDAR-Based Odometry
[**YouTube**](https://youtu.be/3-xLRRT25ys) | **[arXiv](https://arxiv.org/abs/2504.11580)** | **[Website](https://asig-x.github.io/resple_web/)** | **[IEEE RA-L](https://doi.org/10.1109/LRA.2025.3604758)** | **[Demonstrator](https://asig-x.github.io/resple_demonstrator/)**

## What is RESPLE?

RESPLE is a continuous-time state estimator for 6-DoF pose (position + orientation), used here as the backbone for a family of direct LiDAR-based odometry systems: LiDAR-only (LO), LiDAR-Inertial (LIO), and their multi-LiDAR variants (MLO/MLIO).

### Continuous-time trajectory as a B-spline

Instead of representing the trajectory as a sequence of discrete poses — one per sensor measurement, as in most discrete-time filters or pose-graph optimizers — RESPLE represents it as a uniform cubic B-spline over SE(3): a small, fixed number of active control points (a translation and a local orientation increment each) that continuously parameterize position and orientation over a sliding time window. Any sensor measurement can then be related to the trajectory by evaluating the spline — and its analytic derivatives — at that measurement's *exact* timestamp. There is no per-scan or per-message discretization, and no explicit deskewing step: asynchronous, multi-rate sensors (LiDAR points arriving continuously, IMU at high rate, wheel odometry at yet another rate) are all handled the same way, by querying the same continuous trajectory at each one's own timestamp.

### Recursive estimation (IEKF)

The 4 active control points of the spline are corrected online with an Iterated Extended Kalman Filter (IEKF): each new batch of measurements produces a residual and an analytic Jacobian *with respect to the spline control points* (derived from the spline's own basis functions), which are stacked and solved as a single batch update. As the sliding window advances, the oldest control point is retired and a new one added under a constant-velocity motion prior, and its associated process noise is propagated through the filter's covariance.

### Sensor factors

- **LiDAR** (point-to-plane): for each raw point, RESPLE queries the spline for the pose at that point's exact capture time, transforms it into the world frame using the LiDAR's extrinsic calibration, finds its nearest neighbors in an incremental kd-tree (`ikd-Tree`) local map, fits a local plane, and uses the signed point-to-plane distance as the residual.
- **IMU** (accelerometer + gyroscope): the spline's own analytic 2nd derivative (linear acceleration) and angular velocity are compared directly against raw IMU readings — no separate integration step. In LIO mode, accelerometer and gyroscope biases are also estimated as part of the filter state.
- **Wheel/velocity odometry** (optional, tightly-coupled — see [Wheel Odometry](#wheel-odometry) below): the spline's analytic linear and angular velocity are compared against wheel-reported velocity, with adaptive covariance inflation to reject wheel slip.

### Nodes

Two ROS2 nodes make up the pipeline:
- **`RESPLE`** — runs the estimator itself: ingests LiDAR/IMU/wheel-odometry, maintains the local ikd-tree map used for LiDAR association, and publishes the current spline window.
- **`Mapping`** — consumes the spline window, builds the downsampled global map, publishes the trajectory/odometry, and broadcasts the TF tree (`world`, `body`, per-LiDAR frames, and an optional `target_link`/`odom` bridge — see `resple/config/config_*.yaml`).

A third, optional node, **`MapSaving`**, accumulates the dense global map for the whole session (see [Map Saving](#map-saving) below).

![RESPLE demo](doc/demo.gif)

## What This Fork Adds

- **[Wheel Odometry](#wheel-odometry)** — optional, tightly-coupled fusion of wheel/velocity odometry as an IEKF factor, with adaptive covariance inflation to reject wheel slip.
- **[Relocalization](#relocalization)** — opt-in, startup-only initialization against a previously-saved map: a coarse pose guess (config or RViz's "2D Pose Estimate") is refined via ICP before tracking starts.
- **[Covariance](#covariance)** — `odometry` now publishes real pose and twist covariance, propagated from the IEKF's own filter state, plus twist (velocity) itself, previously unpublished.

## Dependencies
Tested with [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) on Ubuntu 24.04
```
sudo apt install libomp-dev libpcl-dev libeigen3-dev
sudo apt install ros-jazzy-pcl*
```

## Compilation
```
cd ~/ros2_ws/src
git clone --recursive https://github.com/paureverte/RESPLE.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select estimate_msgs livox_ros_driver2 resple
```

## Docker

A ready-to-use ROS2 Jazzy development image is provided under `docker/`.

### Build
```bash
cd ~/path/to/src
git clone --recursive https://github.com/paureverte/RESPLE.git
cd RESPLE/docker
./build.sh
```
This builds the `resple:jazzy` image (pass `./build.sh --no-cache` to force a clean rebuild).

### Run

With the image built, run the algorithm in a docker container via `docker compose`.

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

Launch, replacing `<name>` with one of the configs from [Usage](#usage) below:
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

## Usage

Every dataset/sensor setup is launched through the same generic launch file:
```bash
source install/setup.bash
ros2 launch resple resple.launch.py config:=<name>
# Open another terminal and run
source install/setup.bash
ros2 bag play /path/to/bag/
```
Optional launch arguments:
* `rviz:=false` — skip launching RViz (default `true`)
* `publish_static_tf:=true` — also publish an identity `map`→`my_frame` static transform, for configs with no map-frame source of their own (default `false`)
* `map_saving:=true` — also launch the `MapSaving` node (default `false`, see [Map Saving](#map-saving))

Available `<name>` values, each backed by `resple/config/config_<name>.yaml`:

| `config:=` | Sensors | Mode |
|---|---|---|
| `hap360` | Livox HAP360 + IMU | LiDAR-Inertial |
| `hap360_lidaronly` | Livox HAP360 | LiDAR-only |
| `mid360` | Livox Mid360 (PointCloud2) + IMU | LiDAR-Inertial |
| `mid360_custommsg` | Livox Mid360 (livox_ros_driver2 CustomMsg) + IMU | LiDAR-Inertial |
| `ouster` | Ouster + IMU | LiDAR-Inertial |
| `ouster_lidaronly` | Ouster | LiDAR-only |

Each `config_<name>.yaml` is commented inline and grouped into `frames:` (which also holds `target_link`), `imu:`, `lidar:`, `spline:`, `mapping:`, `relocalization:` and `wheel_odometry:` sections. Copy the closest one as a starting point for your own sensor setup, and toggle `if_lidar_only` to switch between LiDAR-only and LiDAR-Inertial for a given sensor.

### LiDAR types

Each entry under `lidar.<name>.lidar_type` selects both the ROS message type RESPLE subscribes to for that LiDAR *and* the point-field parsing/timestamp convention used — the two always go together, you can't mix a driver's actual output format with the wrong `lidar_type`:

| `lidar_type` | ROS message type | Per-point timestamp | Notes |
|---|---|---|---|
| `Ouster` | `sensor_msgs/PointCloud2` (`ouster_ros::Point`) | relative offset (ns) from scan start, field `t` | |
| `Hesai` | `sensor_msgs/PointCloud2` (`hesai_ros::Point`) | absolute epoch **seconds**, field `timestamp` | uses `ring`, not `tag`/`line` |
| `Mid360` | `sensor_msgs/PointCloud2` (`livox_mid360::Point`) | absolute epoch **nanoseconds**, field `timestamp` | driver configured for standard PointCloud2 output (`xfer_format: 1`); has `tag`/`line` fields but does **not** currently filter by them (unlike `LivoxCustomMsg`) |
| `LivoxCustomMsg` | `livox_ros_driver2/msg/CustomMsg` | relative offset (ns) from scan start, field `offset_time` | filters by `tag`/`line`; not tied to any specific sensor model — matches whichever Livox unit your `livox_ros_driver2` publishes CustomMsg for |

If your driver's actual message type/timestamp convention doesn't match the table above (verify with `ros2 topic echo <topic> --once` and compare against a known-good `header.stamp`), pick the closest existing parser as a starting point rather than guessing — a mismatched timestamp convention breaks deskewing silently instead of erroring out.

## Wheel Odometry

RESPLE can optionally fuse wheel/velocity odometry as a **tightly-coupled** IEKF factor: rather than pre-integrating wheel velocity into a separate pose estimate and fusing that, the spline's own analytic linear and angular velocity are evaluated at the wheel measurement's exact timestamp and compared directly against the reported velocity — the same principle already used for the IMU factor.

**Frames.** Let `W` be world, `B` the body/IMU frame (what the spline directly estimates), and `O` the wheel-odometry frame. `R_BO`, `p_BO` is the static extrinsic placing `O` in `B`, calibrated the same way as the LiDAR extrinsic (`q_wb`/`t_wb` in the config, inverted internally exactly like `q_lb`/`t_lb`).

**Predicted velocity.** At the wheel measurement's timestamp $t_k$, the spline gives the world-frame linear velocity $\dot{\mathbf p}_W(t_k)$ and the body-frame angular velocity $\boldsymbol\omega_B(t_k)$ directly (both analytic derivatives of the spline — no finite differencing). The predicted velocity at the wheel frame, accounting for the lever-arm effect of rotation, is:

$$\mathbf v_{\text{pred}}(t_k) = \mathbf R_{BO}^\top \left( \mathbf R_{WB}(t_k)^\top \dot{\mathbf p}_W(t_k) \;-\; \boldsymbol\omega_B(t_k) \times \mathbf p_{BO} \right)$$

**Residual.**

$$\mathbf r = \mathbf v_{\text{meas}} - \mathbf v_{\text{pred}}(t_k)$$

If `use_only_vx: true`, only the forward-axis component $r_x$ is used (useful when lateral/vertical wheel velocity is dominated by non-holonomic noise rather than signal).

**Jacobians.** Both are evaluated analytically against the spline's own basis-function Jacobians for each of the 4 active control points $i$ (translation $\mathbf p_i$ and orientation increment $\delta\boldsymbol\phi_i$) — the same per-knot Jacobians already computed for the LiDAR/IMU factors, no numerical differentiation:

$$\frac{\partial \mathbf r}{\partial \mathbf p_i} = -\, \mathbf R_{BO}^\top \mathbf R_{WB}(t_k)^\top \, J_{\text{vel}, i}$$

$$\frac{\partial \mathbf r}{\partial \delta\boldsymbol\phi_i} = -\, \mathbf R_{BO}^\top \left( [\mathbf v_{\text{body}}]_\times \, J_{\text{ortdel}, i} \;+\; [\mathbf p_{BO}]_\times \, J_{\text{gyro}, i} \right), \qquad \mathbf v_{\text{body}} = \mathbf R_{WB}(t_k)^\top \dot{\mathbf p}_W(t_k)$$

where $[\cdot]_\times$ denotes the skew-symmetric cross-product matrix, and $J_{\text{vel},i}$, $J_{\text{ortdel},i}$, $J_{\text{gyro},i}$ are the spline's basis-function Jacobians for control point $i$.

**Adaptive covariance (slip rejection).** Before the residual is weighted into the IEKF update, its forward-axis magnitude is checked against a threshold; if exceeded, the measurement covariance is inflated so the update trusts the wheel far less for that step (rather than being rejected outright):

$$\mathbf M_w = \mathrm{diag}(\sigma_{vx}^2, \sigma_{vy}^2, \sigma_{vz}^2), \qquad \text{if } |r_x| > \tau_{\text{slip}}: \ \mathbf M_w \mathrel{*}= \lambda_{\text{slip}}$$

**Config.** Disabled by default; enable it per-config under `wheel_odometry:` (see `config_hap360.yaml` or `config_mid360.yaml` for the full commented template):

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

  max_allowed_residual_vx: 0.5           # τ_slip (m/s): residual above which the update is treated as wheel slip
  adaptive_covariance_multiplier: 100.0  # λ_slip: covariance inflation applied when slip is detected
```

## Map Saving

Two ways to save the map as a `.pcd` file. Both save services are `estimate_msgs/srv/SaveMap` (`string path` request, `bool success` + `string message` response) — pass an absolute `path` to save exactly there for that call, or omit it (empty string) to fall back to the path configured in the yaml.

**Option 1 — save RESPLE's local map**

The `RESPLE` node exposes a `/save_map` service that saves the current ikd-tree map (bounded by `mapping.cube_len` — older points get evicted as the sensor moves away) directly to disk:
```bash
ros2 service call /save_map estimate_msgs/srv/SaveMap "{path: '/home/user/maps/my_map.pcd'}"
# or, to use the configured default path instead:
ros2 service call /save_map estimate_msgs/srv/SaveMap "{}"
```
Default path (used when `path` is empty) is `/tmp/resple_map.pcd`, overridable per config via `mapping.pcd_save_path`.

**Option 2 — accumulate and save the full-session dense map**

Launch with `map_saving:=true` to also start the `MapSaving` node, which subscribes to `global_map` (published by `Mapping`) and accumulates every scan into one unbounded dense map for the whole session:
```bash
ros2 launch resple resple.launch.py config:=<name> map_saving:=true
# whenever you want to save it:
ros2 service call /save_global_map estimate_msgs/srv/SaveMap "{path: '/home/user/maps/session_map.pcd'}"
```
Default path (used when `path` is empty) is `/tmp/resple_global_map.pcd`, overridable per config via `mapping.global_pcd_save_path`.

`mapping.ds_map_voxel` controls the voxel size (m) each scan is downsampled to before being added to `global_map`/the accumulated map — independent of `mapping.ds_lm_voxel`, which only affects RESPLE's own local ikd-tree.

To view a saved `.pcd` file:
```bash
pcl_viewer /path/to/output.pcd
```

## Relocalization

By default RESPLE always initializes from scratch: position `(0,0,0)`, gravity-aligned orientation, and a local map built from the first ~100ms of live points. Enabling `relocalization` instead loads a previously-saved map and, on startup only, refines a coarse initial pose guess against it via ICP — after that one-time step, everything continues exactly as usual (no re-relocalization mid-run; if tracking is lost, restart the node).

This is **local refinement, not place recognition**: it corrects a guess that's already roughly right (a few meters/degrees off), it does not search the whole map for where the sensor might be. A guess that's far from the true pose will simply fail to converge rather than silently converging on the wrong place — the node logs an error and falls back to normal from-scratch initialization for the rest of that run (same as `relocalization.enable: false`), instead of crashing. A failure to *load* `map_path` at all (missing/corrupt file) is treated differently and is fatal, since that's a config problem that won't fix itself on retry.

**Map source.** Use `/save_global_map`'s output (see [Map Saving](#map-saving)), not `/save_map`'s — the latter is bounded/pruned by `mapping.cube_len` and won't represent the wider area you're relocalizing into.

Once loaded, the (downsampled) prior map is published once on `prior_map` (`sensor_msgs/PointCloud2`, transient-local — RViz picks it up even if it connects after startup) and shown by default in `config.rviz`, in gray, so you can see where to click "2D Pose Estimate" when `initial_guess: false`.

**Initial guess**, either:
- `initial_guess: true` — a fixed `t0`/`q0` in the config, for a repeated start point (e.g. always the same garage/dock).
- `initial_guess: false` — RViz's built-in "2D Pose Estimate" tool (already wired to `/initialpose` in `config.rviz`, no setup needed): click-drag a rough pose after launch. Note this tool only sets x/y/yaw (z/roll/pitch are always 0) — ICP is relied on to correct the rest, so don't expect a good guess on height/tilt from it alone.

```yaml
relocalization:
  enable: true
  map_path: "/home/user/maps/session_map.pcd"   # /save_global_map output
  initial_guess: false                          # or true, with t0/q0 below
  t0: [0.0, 0.0, 0.0]
  q0: [1.0, 0.0, 0.0, 0.0]                       # w, x, y, z
  icp_max_iterations: 50
  icp_max_corr_dist: 1.0                         # max ICP point-pair distance, unit: m
  icp_fitness_threshold: 0.5                     # reject (fatal-exit) if ICP's fitness exceeds this
```

## Covariance

`Mapping`'s `odometry` topic (`nav_msgs/msg/Odometry`) publishes real `pose.covariance` and a full `twist` (body-frame linear + angular velocity, with `twist.covariance`) — not placeholders. Both are propagated from the IEKF's own live control-point covariance, using the same analytic Jacobians already used to build the LiDAR/IMU/wheel factors' own residuals, evaluated at the current spline tip.

One approximation worth knowing: the pose `odometry` actually publishes lags the live estimate by design (`Mapping` only publishes settled trajectory — see [Nodes](#nodes)). The covariance/twist transmitted correspond to RESPLE's estimate at message-send time; `Mapping` uses the most recently received values as a running proxy rather than re-deriving covariance for that exact, slightly older published pose.

## Contributors
Ziyu Cao (Email: ziyu.cao@liu.se)

William Talbot (Email: wtalbot@ethz.ch)

Kailai Li (Email: kailai.li@rug.nl)

Pau Reverté (Email: pau.reverte@eurecat.org)

## Credits
Thanks for [SFUISE](https://github.com/ASIG-X/SFUISE), [ikd-Tree](https://github.com/hku-mars/ikd-Tree), [FAST-LIO](https://github.com/hku-mars/FAST_LIO), [Livox-SDK](https://github.com/Livox-SDK), and [basalt](https://gitlab.com/VladyslavUsenko/basalt).

## License
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.
