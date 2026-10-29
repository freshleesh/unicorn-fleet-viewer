# unicorn-fleet-viewer

Main-PC side of the UNICORN Racing expo fleet: three 1/10 cars on one track, one driving
itself and two driven by visitors with game pads. This repository is everything the
**viewing / control PC** runs, plus the fleet scripts the cars run on top of their racing
stack ([`car/`](#car-side-car): relay, agent ring planner, start scripts). The PC only needs
the cars on the same Wi-Fi and running their fleet relay (see [Topics](#topics-shared-with-the-cars)).

- RViz fleet view: 2D track map, 3D point-cloud map, each car's body, pose, planned path,
  live scan and camera
- Driver screens: chase view behind one car + its camera, side by side
- Per-car **Pose** tools (car colour) to set each car's localization start pose
- **Autodrive** panel: per-car AUTO / OFF, ALL AUTO / ALL OFF, per-car speed cap (1–8 m/s)
- Game pad → car bridge
- Battery watch over ssh

## Requirements

- Ubuntu 24.04, ROS 2 Jazzy (`ros-jazzy-desktop`)
- On the same Wi-Fi as the cars

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-nav2-map-server \
                 ros-jazzy-nav2-lifecycle-manager ros-jazzy-joy python3-numpy
```

## Install

```bash
mkdir -p ~/unicorn_fleet_ws/src && cd ~/unicorn_fleet_ws/src
git clone https://github.com/freshleesh/unicorn-fleet-viewer.git
cd ~/unicorn_fleet_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

## Run

Every shell that talks to the cars sources the fleet environment first:

```bash
source ~/unicorn_fleet_ws/install/fleet_viewer/share/fleet_viewer/env/fleet_env.sh
```

It sources ROS + the workspace and puts CycloneDDS on the shared fleet domain (`ROS_DOMAIN_ID=1`)
bound to this PC's Wi-Fi interface, with multicast for discovery only (Wi-Fi multicast drops
camera frames). The interface is found automatically; override it with `FLEET_IFACE=<name>`.

**Fleet view**

```bash
ros2 launch fleet_viewer view_fleet.launch.py                          # bundled track
ros2 launch fleet_viewer view_fleet.launch.py map:=/path/to/track.yaml # another map (+ track.pcd next to it)
```

Closing RViz stops the map servers too.

- Toolbar **Pose: nuc1 / nuc4 / nuc5** — click-drag where the car is and the way it faces.
  The car's live scan (turn on `<car> scan` in Displays) should then sit on the 3D map walls.
- **Autodrive** panel
  - AUTO / OFF per car; ALL AUTO / ALL OFF. AUTO hands the car to its planner, OFF stops it
    (speed 0, visitor's pad can take over). The button shows what was last sent, not what
    the car reports — a visitor pressing LB on the pad takes the car back without the panel knowing.
  - Slider: speed cap per car, 1.0–8.0 m/s. It is the planner's speed **and** the pad's
    full-stick speed. Sent when RViz opens, so opening RViz sets every car to the slider values.
    `File → Save Config` keeps the slider positions.

**Driver screens** (one per visitor car, on any PC on the fleet Wi-Fi)

```bash
ros2 launch fleet_viewer driver_view.launch.py car:=nuc1 x:=1920   # e.g. main PC, second monitor
ros2 launch fleet_viewer driver_view.launch.py car:=nuc5           # e.g. another PC
```

Left half: chase view behind the car (track, walls, every car, its planner path); right half:
the car's camera. `x`/`y` place the window (pick the monitor), `width`/`height` default to
1920×1080. Each screen serves its own map copy and TF under `/driver_<car>/`, so the screens
and the main view don't interfere wherever they run. The chase camera follows the frame
`<car>` that `odom_tf.py` makes from `/<car>/odom`. Closing RViz stops the rest.

**Game pads** (one per car)

```bash
ls /dev/input/js*                                                   # js0, js1, ...
ros2 launch fleet_viewer joy.launch.py car:=nuc1 device_id:=0
ros2 launch fleet_viewer joy.launch.py car:=nuc5 device_id:=1
```

Hold **LB** to drive (left stick: speed, right stick: steering). **RB** hands the car to autodrive.

**Battery watch** (optional; needs passwordless `ssh nuc1` etc. to the cars)

```bash
~/unicorn_fleet_ws/install/fleet_viewer/share/fleet_viewer/tools/battery_watch.sh
```

Checks each car's VESC input voltage every 5 minutes (4S LiPo) and prints only when a pack
is below 15.0 V (swap soon) or 14.6 V (swap now). Full log: `~/.ros/fleet_battery.log`.
`CARS="nuc1 nuc5"` picks the cars.

## Car side (`car/`)

Scripts the cars run on top of the UNICORN racing stack. They are not a ROS package
(`COLCON_IGNORE`); copy the folder to each car as `~/expo_config` (the cars have no internet):

```bash
scp -r car/. nuc1:expo_config/        # same for nuc4, nuc5
```

The car needs the racing stack built at `~/unicorn_ws` (`stack_master`, `kiss_icp_localization`,
`controller.geofence`; override the setup script with `UNICORN_SETUP=`), the track map at
`~/maps/<map>/<map>.{pcd,yaml,pgm}` and `scipy` + `scikit-image` for the planner.
Apply the speed-cap patch to the stack once (lets the panel slider cap the pad):

```bash
cd ~/unicorn_ws/src/unicorn-racing-stack && patch -p1 < ~/expo_config/patches/simple_mux_max_speed.patch
cd ~/unicorn_ws && colcon build --packages-select stack_master
```

`fleet_env.sh` picks the car from the hostname (`nuc1` / `nuc4` / `nuc5`, override `FLEET_CAR=`):
private domain 11 / 14 / 15 on localhost only, shared domain 1 on Wi-Fi.

**Start** — one terminal (or tmux pane) each, on the car, in this order:

```bash
~/expo_config/start_low_level.sh geofence_map:=run_1016_1436   # VESC, lidar, camera, mux
~/expo_config/start_kiss.sh run_1016_1436                      # localization; then "Pose: <car>" in RViz
~/expo_config/start_relay.sh                                   # appears on the PC as /<car>/...
~/expo_config/start_planner.sh run_1016_1436                   # agent driver (autonomous / guest practice)
```

Without the planner the car is pad-only. With it, **AUTO** in the Autodrive panel (or RB on the pad)
hands the car to the planner; OFF or LB takes it back. Planner tuning goes as ROS args,
e.g. `start_planner.sh run_1016_1436 -p follow_gap:=0.6`.

The planner drives round the ring for ever: every 0.2 s an A* search from the car to the
centreline point a third of a lap ahead, one-way (never against the track direction), cost =
walls + other cars' footprints + distance from the previous path; pure pursuit follows it at
the panel's speed cap and slows down only if the road is fully blocked by a car.

**Fix-ups**

| | |
|---|---|
| `start_camera.sh` | restart only the OAK-D (e.g. `X_LINK_ERROR`); log `~/.ros/oak.log` |
| `restart_mux.sh` | restart only the mux with its current params, e.g. after editing the geofence map |
| `stop_all.sh` | stop everything above |

## Topics shared with the cars

All on domain 1. Each car runs its own stack on a private domain and a relay copies these across.

| Topic | Type | Direction | |
|---|---|---|---|
| `/<car>/odom` | nav_msgs/Odometry | car → PC | localization, 30 Hz, frame `map` |
| `/<car>/marker` | visualization_msgs/MarkerArray | car → PC | body box + name |
| `/<car>/path` | nav_msgs/Path | car → PC | planner path, 5 Hz |
| `/<car>/image/compressed` | sensor_msgs/CompressedImage | car → PC | camera (JPEG) |
| `/<car>/scan` | sensor_msgs/PointCloud2 | car → PC | live lidar scan in `map` |
| `/<car>/drive` | ackermann_msgs/AckermannDriveStamped | car → PC | command actually sent to the motor |
| `/<car>/vesc/core` | vesc_msgs/VescStateStamped | car → PC | VESC state |
| `/<car>/joy` | sensor_msgs/Joy | PC → car | pad / Autodrive panel |
| `/<car>/initialpose` | geometry_msgs/PoseWithCovarianceStamped | PC → car | Pose tools |
| `/<car>/max_speed` | std_msgs/Float32 (transient local) | PC → car | speed cap |
| `/map`, `/map_cloud` | OccupancyGrid, PointCloud2 | PC | track maps |

Cars: `nuc1` (orange), `nuc4` (blue), `nuc5` (green). The names and colours live in
`fleet_rviz_tools/src/*.cpp` and `fleet_viewer/rviz/fleet.rviz`.

## Packages

- `fleet_rviz_tools` — RViz plugins: `PoseNuc1/4/5` tools, `AutodrivePanel`
- `fleet_viewer` — launch files (main view, driver screens, pads), RViz configs, bundled map, `pcd_publisher.py`, `odom_tf.py`, env script, battery tools
- `car/` — not a package: scripts the cars run (env, relay, ring planner, start/stop), mux patch
