# ROS2 Workspace Guide (Docker + Network Monitoring)

This guide shows how to use this workspace step by step:
- Start Docker container
- Build ROS2 workspace
- Run launch files or nodes
- Visualize with Foxglove / RViz2
- SLAM mapping
- Autonomous navigation

---

## 1) Prerequisites

On the host (outside container):

```bash
docker --version
```

Make sure serial devices exist before starting the container:

```bash
ls -l /dev/ttyUSB0
ls -l /dev/ttyUSB1
```

- `/dev/ttyUSB0`: ESP32 (required by `run_ros.sh`)
- `/dev/ttyUSB1`: LiDAR (optional; script continues if missing)

---

## 2) Build Docker Image (First Time or After Dockerfile Change)

From host:

```bash
cd ~
docker build -t ros_robot:humble .
```

This image installs: `rviz2`, `rqt`, `foxglove_bridge`, `pyserial`.

> Nav2 and slam_toolbox were installed manually inside the container via `apt` and survive reboots.
> They would only be lost if the container is deleted (`docker rm ros_humble`). To make them permanent, add them to the Dockerfile.

---

## 3) Start ROS Container

From host:

```bash
cd ~
./run_ros.sh
```

Behavior:
- Creates container `ros_humble` if it does not exist yet
- Starts it if it exists but is stopped
- Attaches shell if it is already running

Container settings:
- Host networking (`--net=host`)
- X11 forwarding for GUI apps
- Bind mount: `~/ros2_ws` → `/ros2_ws`

---

## 4) Build Workspace Inside Container

```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

Run `source /ros2_ws/install/setup.bash` in every new container terminal.

---

## 5) Run the Full Robot Stack (Recommended)

`amr_common.launch.py` starts everything in one command:
- ESP32 bridge node
- LiDAR driver
- Odometry node
- Static TF (`base_link → laser_frame`)
- Foxglove bridge (port 8765)

```bash
ros2 launch amr_common amr_common.launch.py
```

Open Foxglove from another device: `ws://<ROBOT_IP>:8765`

Find robot IP:
```bash
hostname -I
```

---

## 6) SLAM — Build a Map

SLAM requires the full robot stack running first (section 5), then launch slam_toolbox in a second terminal.

**Terminal 1:**
```bash
ros2 launch amr_common amr_common.launch.py
```

**Terminal 2:**
```bash
ros2 launch amr_common slam.launch.py
```

Drive the robot slowly around the area (max 0.2 m/s — lidar runs at 7 Hz).
Watch the map build live in Foxglove by subscribing to `/map`.

**Save the map when done:**
```bash
ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/maps/my_map
```

This creates `/ros2_ws/src/maps/my_map.pgm` and `/ros2_ws/src/maps/my_map.yaml`.

---

## 7) Navigation — Drive to a Goal

Navigation requires the full robot stack (section 5) plus the Nav2 stack.

**Terminal 1:**
```bash
ros2 launch amr_common amr_common.launch.py
```

**Terminal 2:**
```bash
ros2 launch amr_common nav.launch.py map:=/ros2_ws/src/maps/my_map.yaml
```

Wait for all nodes to report active in the logs.

### 7.1 Set Initial Pose

Tell AMCL where the robot is on the map. Replace `x` and `y` with the robot's actual position:

```bash
ros2 topic pub -r 10 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]
  }
}" 
```

> **Important:** Use `-r 10` (publish at 10 Hz) and let it run for 2 seconds, then Ctrl+C.
> Publishing `--once` is unreliable — AMCL may miss a single message.

Confirm AMCL accepted it — look for this in the nav launch terminal:
```
[amcl]: Setting pose: X.XXX Y.YYY Z.ZZZ
```

### 7.2 Send a Navigation Goal

After AMCL is localized, send a goal position on the map:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 1.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

Replace `x` and `y` with a valid goal coordinate visible on your map.

> If `Waiting for at least 1 matching subscription(s)...` appears, use `-r 10` for 2 seconds instead of `--once`.

The robot will plan a path and drive autonomously to the goal.

---

## 8) TF Frames

| Transform | Published by |
|---|---|
| `odom → base_link` | `odom_node` (esp32_bridge) |
| `base_link → laser_frame` | `static_tf.launch.py` (amr_common) |
| `map → odom` | `amcl` (during navigation) |

Check TF chain:
```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_tools view_frames   # saves frames.pdf
```

---

## 9) Useful Debug Commands

```bash
# List all active nodes and topics
ros2 node list
ros2 topic list

# Check if scan and odom are publishing
ros2 topic hz /scan
ros2 topic hz /odom

# Echo odom without covariance noise
ros2 topic echo /odom --no-arr

# Check Nav2 lifecycle states
for node in amcl map_server planner_server controller_server bt_navigator; do
  echo -n "$node: "; ros2 lifecycle get /$node
done

# Check what map_server actually loaded
ros2 param get /map_server yaml_filename
```

---

## 10) Stop / Restart

From host:

```bash
docker stop ros_humble
docker start -ai ros_humble
```

Open extra shell in running container:

```bash
docker exec -it ros_humble bash
```

---

## Troubleshooting

**No ROS packages found:**
```bash
source /ros2_ws/install/setup.bash
```

**Nav2 nodes stuck in `unconfigured`:**
The lifecycle manager activates them in order automatically. If it fails, check the launch terminal for `FATAL` lines — usually a wrong plugin name or missing action server.

**AMCL not localizing:**
- Particles scattered on map = initial pose not set, or set in wrong location
- Drive the robot slowly — the particle filter converges with motion
- Check that `/scan` overlays on the map walls in Foxglove

**No serial permissions:**
```bash
sudo usermod -aG dialout $USER
# then log out and back in
```

**RViz is slow:**
Software GL is used (`LIBGL_ALWAYS_SOFTWARE=1`) for ARM compatibility. Use Foxglove instead for visualization.
