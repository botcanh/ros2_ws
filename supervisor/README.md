# AMR Supervisor — Pi Setup Guide

Always-on HTTP API on the Pi host (port **8000**) that turns the robot system
on/off and switches modes. It also serves the **browser control app** (`web/`).
Works together with **rosbridge** (port **9090**, inside the container) which
carries live ROS data.

## Control app (laptop)

Open **`http://<PI_IP>:8000/?key=<AMR_API_KEY>`** in any browser on the same
network (the key is remembered after the first visit; change it later with
"API key…"). No install needed on the laptop.

| Mode | What runs | What you can do |
|------|-----------|-----------------|
| Power ON / MANUAL | `stack` (bridge, lidar, odom, rosbridge) | drive with joystick or W/A/S/D, see cmd_vel + odom + laser |
| MAPPING | `stack` + `slam` | drive, watch the live map, **Save map** as `name` or `folder/name` → `~/ros2_ws/src/maps/` |
| AUTO | `stack` + `nav` (map_server, amcl, Nav2) | pick a map → **Load**, **Set initial pose**, **Set goal**, Cancel |

E-STOP (top right) cancels any Nav2 goal and sends zero velocity. Touching the
drive controls in AUTO also cancels the active goal. Releasing the joystick /
keys (or closing the tab) stops the robot within 0.5 s (bridge timeout).

Everything below runs **on the Raspberry Pi**.

---

## 1) Pull this code onto the Pi

```bash
cd ~/ros2_ws/src
git pull
```

## 2) Install rosbridge inside the container

```bash
docker start ros_humble          # if not already running
docker exec -it ros_humble bash -c "apt update && apt install -y ros-humble-rosbridge-suite"
```

Make it permanent by adding this line to the Dockerfile in `~` (next to the
other apt installs), so it survives a container re-create:

```dockerfile
RUN apt-get update && apt-get install -y ros-humble-rosbridge-suite
```

## 3) Rebuild the workspace (launch file changed)

`amr_common.launch.py` now also starts `rosbridge_websocket` on port 9090.

```bash
docker exec -it ros_humble bash -c "source /opt/ros/humble/setup.bash && cd /ros2_ws && colcon build --packages-select amr_common"
```

## 4) Install supervisor dependencies (on the Pi host, NOT in the container)

```bash
sudo apt update
sudo apt install -y python3-fastapi python3-uvicorn python3-psutil
```

The supervisor calls `docker`, so your user must be in the docker group
(check with `groups`; add with `sudo usermod -aG docker $USER` + re-login).

## 5) Create the API key

```bash
openssl rand -hex 16             # copy the output
sudo tee /etc/amr-supervisor.env > /dev/null <<'EOF'
AMR_API_KEY=<paste-the-key-here>
EOF
sudo chmod 600 /etc/amr-supervisor.env
```

You will enter this same key in the control app (or pass it once as `?key=`).

## 6) Install the systemd service

First edit `amr-supervisor.service`: replace `pi` in `User=` and the two
`/home/pi/...` paths with your actual username (`whoami`).

```bash
cd ~/ros2_ws/src/supervisor
nano amr-supervisor.service      # fix User= and paths
sudo cp amr-supervisor.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now amr-supervisor
systemctl status amr-supervisor  # should say "active (running)"
```

Logs: `journalctl -u amr-supervisor -f`

## 7) Verify on the Pi

```bash
curl http://localhost:8000/ping
# {"ok":true,"service":"amr-supervisor"}

KEY=<your-key>
curl -H "X-API-Key: $KEY" http://localhost:8000/status
curl -X POST -H "X-API-Key: $KEY" http://localhost:8000/system/start
curl -X POST -H "X-API-Key: $KEY" http://localhost:8000/launch/stack
curl -H "X-API-Key: $KEY" http://localhost:8000/logs/stack
```

After `/launch/stack` succeeds, rosbridge should be listening:

```bash
ss -tlnp | grep -E '9090|8765'
```

## 8) Verify from the Windows PC

```powershell
curl.exe http://<PI_IP>:8000/ping
pip install roslibpy
```

```python
# test_rosbridge.py — proves the full ROS data path
import roslibpy
ros = roslibpy.Ros(host='<PI_IP>', port=9090)
ros.run()
print('connected:', ros.is_connected)
topic = roslibpy.Topic(ros, '/odom', 'nav_msgs/Odometry')
topic.subscribe(lambda msg: print('odom x =', msg['pose']['pose']['position']['x']))
import time; time.sleep(5)
ros.terminate()
```

---

## API reference

All endpoints except `/ping` require header `X-API-Key: <key>`.

| Method | Path                 | What it does                                      |
|--------|----------------------|---------------------------------------------------|
| GET    | `/ping`              | Connectivity check (no auth)                      |
| GET    | `/status`            | Container state, running launches, CPU/mem/temp   |
| POST   | `/system/start`      | `docker start ros_humble`                         |
| POST   | `/system/stop`       | Graceful launch shutdown, then `docker stop`      |
| POST   | `/launch/stack`      | `ros2 launch amr_common amr_common.launch.py`     |
| POST   | `/launch/slam`       | `ros2 launch amr_common slam.launch.py`           |
| POST   | `/launch/nav`        | nav launch; JSON body `{"map": "..."}` optional   |
| POST   | `/launch/{name}/stop`| Stop that launch (SIGINT, then SIGKILL)           |
| POST   | `/mode`              | `{"mode": "off|manual|mapping|auto", "map": "name"}` — starts/stops launches to reach that mode |
| GET    | `/maps`              | Maps under `~/ros2_ws/src/maps` (`name` = path without `.yaml`) |
| POST   | `/maps/save`         | `{"name": "lab/floor1", "overwrite": false}` — map_saver_cli while SLAM runs |
| GET    | `/logs/{name}?lines=100` | Tail of that launch's log file (rewritten on each start) |
| GET    | `/`                  | Redirects to the control app (`/app/`, no key needed to load it) |

Launch output is written to `supervisor/logs/<name>.log` (visible from both
host and container via the bind mount).

## Troubleshooting

- **Service fails at start with "AMR_API_KEY is not set"** — step 5 not done,
  or `EnvironmentFile` path wrong.
- **`/system/start` returns "container does not exist"** — the supervisor only
  starts/stops the existing container; create it once with `./run_ros.sh`.
- **`/launch/stack` returns `running: false`** — check `GET /logs/stack`;
  usual causes are a missing `colcon build` (step 3) or `/dev/ttyUSB0` absent.
- **Windows can't reach port 8000/9090** — both services bind 0.0.0.0 and the
  container uses host networking, so check the Pi's firewall (`sudo ufw status`)
  and that both machines are on the same network.
