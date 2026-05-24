# ROS2 Workspace Guide (Docker + Network Monitoring)

This guide shows how to use this workspace step by step:
- Start Docker container
- Build ROS2 workspace
- Run launch files or nodes
- Visualize with RViz2
- View ROS topics from another device over network

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

## 2) Build Docker Image (First Time or After Dockerfile Change)

From host:

```bash
cd ~
docker build -t ros_robot:humble .
```

This image already installs:
- `rviz2`
- `rqt`
- `foxglove_bridge`

## 3) Start ROS Container

From host:

```bash
cd ~
./run_ros.sh
```

Behavior of script:
- Creates container `ros_humble` if not created yet
- Starts it if it exists but is stopped
- Attaches shell if it is already running

Container settings include:
- Host networking (`--net=host`)
- X11 forwarding for GUI apps (RViz)
- Bind mount: `~/ros2_ws` -> `/ros2_ws`

## 4) Build Workspace Inside Container

Inside container shell:

```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

Tip: run `source /ros2_ws/install/setup.bash` in every new container terminal.

## 5) Demonstration: Run LiDAR Node + Bridge Node + Odom Node

This is the exact runtime flow for your robot stack.

### 5.1 Open 3 terminals into the same running container

On host:

```bash
docker exec -it ros_humble bash
docker exec -it ros_humble bash
docker exec -it ros_humble bash
```

In each terminal, source once:

```bash
source /ros2_ws/install/setup.bash
```

### 5.2 Terminal A: Start LiDAR node

```bash
ros2 launch hclidar_driver_ros2 hclidar_launch.py
```

Notes:
- LiDAR params are loaded from `hclidar.yaml`
- Default serial from that file is `/dev/ttyUSB1`

Quick check (Terminal D or another shell):

```bash
source /ros2_ws/install/setup.bash
ros2 topic echo /scan
```

### 5.3 Terminal B: Start ESP32 bridge node

```bash
ros2 run esp32_bridge bridge_node
```

What it does:
- Subscribes `/cmd_vel`
- Sends motor command to ESP32 on `/dev/ttyUSB0`
- Publishes encoder ticks on `/encoder_ticks`

Quick check:

```bash
ros2 topic echo /encoder_ticks
```

### 5.4 Terminal C: Start odom node

```bash
ros2 run esp32_bridge odom_node
```

What it does:
- Subscribes `/encoder_ticks`
- Publishes `/odom`
- Broadcasts TF `odom -> base_link`

Quick check:

```bash
ros2 topic echo /odom
```

### 5.5 Send movement command (to test full chain)

From any extra container terminal:

```bash
source /ros2_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 5
```

Expected chain:
- `bridge_node` receives `/cmd_vel`
- ESP32 returns encoder ticks
- `odom_node` publishes `/odom`

### 5.6 Optional single-launch approach for LiDAR + RViz

```bash
ros2 launch hclidar_driver_ros2 hclidar_launch_rviz.py
```

Use RViz display topic `/scan` and set Reliability policy to Best Effort if needed.

## 7) Use RViz2 in Docker

Inside container:

```bash
source /ros2_ws/install/setup.bash
rviz2
```

If GUI fails:
- On host run `xhost +local:docker`
- Ensure `DISPLAY` is set on host: `echo $DISPLAY`

## 8) View Topics Over Network (Browser/Internet LAN)

Best option in this image: Foxglove WebSocket bridge.

Inside container:

```bash
source /ros2_ws/install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

From another device on same network:
1. Open https://app.foxglove.dev
2. Create a new connection using WebSocket
3. Enter:

```text
ws://<ROBOT_IP>:8765
```

Find robot IP on host:

```bash
hostname -I
```

Because container uses host network, the same host IP is used for bridge access.

## 9) Basic Topic Debug Commands

Inside container:

```bash
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel
ros2 node list
ros2 service list
```

## 10) Stop/Restart

From host:

```bash
docker stop ros_humble
docker start -ai ros_humble
```

Open extra shell in running container:

```bash
docker exec -it ros_humble bash
```

## Troubleshooting

Container starts but no ROS packages found:
- Run `source /ros2_ws/install/setup.bash`

`ros2 launch foxglove_bridge ...` fails:
- Verify package exists: `ros2 pkg list | grep foxglove_bridge`
- Rebuild image if Dockerfile changed

No serial permissions:
- Add your host user to `dialout`, then relogin:

```bash
sudo usermod -aG dialout $USER
```

RViz is slow:
- This setup uses software GL (`LIBGL_ALWAYS_SOFTWARE=1`) for compatibility.

