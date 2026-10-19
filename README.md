# unicorn-fleet-viewer

Main-PC side of the UNICORN Racing expo fleet: three 1/10 cars on one track, one driving
itself and two driven by visitors with game pads. This repository is everything the
**viewing / control PC** runs. The cars are managed separately; this only needs them to
be on the same Wi-Fi and running their fleet relay (see [Topics](#topics-shared-with-the-cars)).

- RViz fleet view: 2D track map, 3D point-cloud map, each car's body, pose, planned path,
  live scan and camera
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
- `fleet_viewer` — launch files, RViz config, bundled map, `pcd_publisher.py`, env script, battery tools
