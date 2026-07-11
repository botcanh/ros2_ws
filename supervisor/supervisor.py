#!/usr/bin/env python3
"""
AMR Supervisor — always-on HTTP API on the Raspberry Pi host.

Lets a remote client (Windows control app) manage the robot stack:
  - start/stop the ROS Docker container
  - start/stop launch files inside the container
  - read host health and launch logs

Runs OUTSIDE the container (systemd service, see amr-supervisor.service).
All endpoints except /ping require the X-API-Key header.

Environment variables:
  AMR_API_KEY      required, shared secret for X-API-Key
  AMR_CONTAINER    docker container name        (default: ros_humble)
  AMR_DEFAULT_MAP  map yaml path in container   (default: /ros2_ws/src/maps/my_map.yaml)
"""

import os
import shlex
import subprocess
import time
from pathlib import Path
from typing import Optional

import psutil
from fastapi import Depends, FastAPI, HTTPException, Header
from pydantic import BaseModel

API_KEY   = os.environ.get('AMR_API_KEY', '')
CONTAINER = os.environ.get('AMR_CONTAINER', 'ros_humble')
DEFAULT_MAP = os.environ.get('AMR_DEFAULT_MAP', '/ros2_ws/src/maps/my_map.yaml')

# This file lives in ~/ros2_ws/src/supervisor on the host, which the container
# sees as /ros2_ws/src/supervisor — so logs written inside the container are
# readable here directly.
LOG_DIR_HOST      = Path(__file__).resolve().parent / 'logs'
LOG_DIR_CONTAINER = '/ros2_ws/src/supervisor/logs'

ROS_SETUP = 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash'

# name -> (ros2 launch command, pgrep pattern identifying it)
LAUNCHES = {
    'stack': ('ros2 launch amr_common amr_common.launch.py', 'amr_common.launch.py'),
    'slam':  ('ros2 launch amr_common slam.launch.py',       'slam.launch.py'),
    'nav':   ('ros2 launch amr_common nav.launch.py map:={map}', 'nav.launch.py'),
}

if not API_KEY:
    raise SystemExit('AMR_API_KEY is not set — refusing to start. '
                     'Set it in /etc/amr-supervisor.env')

app = FastAPI(title='AMR Supervisor')


def require_key(x_api_key: str = Header(default='')):
    if x_api_key != API_KEY:
        raise HTTPException(status_code=401, detail='invalid or missing X-API-Key')


def run(cmd: list, timeout: float = 15.0) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)


def docker_exec(shell_cmd: str, detach: bool = False, timeout: float = 15.0):
    cmd = ['docker', 'exec']
    if detach:
        cmd.append('-d')
    cmd += [CONTAINER, 'bash', '-c', shell_cmd]
    return run(cmd, timeout=timeout)


def container_status() -> str:
    r = run(['docker', 'inspect', '-f', '{{.State.Status}}', CONTAINER])
    return r.stdout.strip() if r.returncode == 0 else 'not_found'


def launch_running(pattern: str) -> bool:
    r = docker_exec(f'pgrep -f {shlex.quote(pattern)} > /dev/null && echo yes || echo no')
    return r.returncode == 0 and 'yes' in r.stdout


def stop_launch(pattern: str, wait_s: float = 5.0):
    """SIGINT the launch process (graceful ROS shutdown), escalate to SIGKILL."""
    docker_exec(f'pkill -INT -f {shlex.quote(pattern)} || true')
    deadline = time.time() + wait_s
    while time.time() < deadline:
        if not launch_running(pattern):
            return
        time.sleep(0.5)
    docker_exec(f'pkill -KILL -f {shlex.quote(pattern)} || true')


class NavRequest(BaseModel):
    map: Optional[str] = None


@app.get('/ping')
def ping():
    """Unauthenticated connectivity check."""
    return {'ok': True, 'service': 'amr-supervisor'}


@app.get('/status', dependencies=[Depends(require_key)])
def status():
    cstat = container_status()
    launches = {}
    if cstat == 'running':
        launches = {name: launch_running(pat) for name, (_, pat) in LAUNCHES.items()}

    temp_c = None
    try:
        temp_c = int(Path('/sys/class/thermal/thermal_zone0/temp').read_text()) / 1000.0
    except (OSError, ValueError):
        pass

    return {
        'container': cstat,
        'launches': launches,
        'host': {
            'cpu_percent': psutil.cpu_percent(interval=0.2),
            'mem_percent': psutil.virtual_memory().percent,
            'disk_percent': psutil.disk_usage('/').percent,
            'temp_c': temp_c,
        },
    }


@app.post('/system/start', dependencies=[Depends(require_key)])
def system_start():
    cstat = container_status()
    if cstat == 'not_found':
        raise HTTPException(500, f'container "{CONTAINER}" does not exist — '
                                 'create it once with run_ros.sh on the Pi')
    if cstat != 'running':
        r = run(['docker', 'start', CONTAINER], timeout=30)
        if r.returncode != 0:
            raise HTTPException(500, f'docker start failed: {r.stderr.strip()}')
    return {'container': container_status()}


@app.post('/system/stop', dependencies=[Depends(require_key)])
def system_stop():
    if container_status() == 'running':
        # Graceful: SIGINT any ros2 launch first so nodes shut down cleanly
        # (esp32_bridge zeroes the motors in its shutdown handler).
        stop_launch('ros2 launch')
        r = run(['docker', 'stop', CONTAINER], timeout=40)
        if r.returncode != 0:
            raise HTTPException(500, f'docker stop failed: {r.stderr.strip()}')
    return {'container': container_status()}


@app.post('/launch/{name}', dependencies=[Depends(require_key)])
def launch_start(name: str, body: Optional[NavRequest] = None):
    if name not in LAUNCHES:
        raise HTTPException(404, f'unknown launch "{name}" — use one of {list(LAUNCHES)}')
    if container_status() != 'running':
        raise HTTPException(409, 'container is not running — call /system/start first')

    cmd_tpl, pattern = LAUNCHES[name]
    if launch_running(pattern):
        return {'launch': name, 'running': True, 'note': 'already running'}

    map_path = (body.map if body and body.map else DEFAULT_MAP)
    launch_cmd = cmd_tpl.format(map=map_path)

    log_file = f'{LOG_DIR_CONTAINER}/{name}.log'
    docker_exec(f'mkdir -p {LOG_DIR_CONTAINER}')
    docker_exec(f'{ROS_SETUP} && {launch_cmd} >> {log_file} 2>&1', detach=True)

    time.sleep(2.0)  # give it a moment to either come up or crash
    running = launch_running(pattern)
    return {'launch': name, 'running': running,
            'log': f'/logs/{name}' if not running else None}


@app.post('/launch/{name}/stop', dependencies=[Depends(require_key)])
def launch_stop(name: str):
    if name not in LAUNCHES:
        raise HTTPException(404, f'unknown launch "{name}" — use one of {list(LAUNCHES)}')
    _, pattern = LAUNCHES[name]
    if container_status() == 'running':
        stop_launch(pattern)
    return {'launch': name, 'running': launch_running(pattern)
            if container_status() == 'running' else False}


@app.get('/logs/{name}', dependencies=[Depends(require_key)])
def logs(name: str, lines: int = 100):
    if name not in LAUNCHES:
        raise HTTPException(404, f'unknown launch "{name}" — use one of {list(LAUNCHES)}')
    log_file = LOG_DIR_HOST / f'{name}.log'
    if not log_file.exists():
        return {'launch': name, 'lines': []}
    content = log_file.read_text(errors='ignore').splitlines()
    return {'launch': name, 'lines': content[-lines:]}
