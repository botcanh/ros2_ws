#!/usr/bin/env python3
"""
AMR Supervisor — always-on HTTP API on the Raspberry Pi host.

Lets a remote client (the browser control app in web/, served at /) manage
the robot stack:
  - start/stop the ROS Docker container
  - start/stop launch files inside the container, or switch whole modes
    (off / manual / mapping / auto) in one call
  - list and save maps
  - read host health and launch logs

Runs OUTSIDE the container (systemd service, see amr-supervisor.service).
All /api-style endpoints except /ping require the X-API-Key header; the
static web app itself is public and sends the key with every call.

Environment variables:
  AMR_API_KEY      required, shared secret for X-API-Key
  AMR_CONTAINER    docker container name        (default: ros_humble)
  AMR_DEFAULT_MAP  map yaml path in container   (default: /ros2_ws/src/maps/my_map.yaml)
  AMR_MAPS_DIR     maps folder on the host      (default: ~/ros2_ws/src/maps next to this file)
"""

import os
import re
import shlex
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

import psutil
from fastapi import Depends, FastAPI, HTTPException, Header
from fastapi.responses import RedirectResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

API_KEY   = os.environ.get('AMR_API_KEY', '')
CONTAINER = os.environ.get('AMR_CONTAINER', 'ros_humble')
DEFAULT_MAP = os.environ.get('AMR_DEFAULT_MAP', '/ros2_ws/src/maps/my_map.yaml')

# This file lives in ~/ros2_ws/src/supervisor on the host, which the container
# sees as /ros2_ws/src/supervisor — so logs written inside the container are
# readable here directly. Same for the maps folder.
HERE              = Path(__file__).resolve().parent
LOG_DIR_HOST      = HERE / 'logs'
LOG_DIR_CONTAINER = '/ros2_ws/src/supervisor/logs'
MAPS_DIR_HOST      = Path(os.environ.get('AMR_MAPS_DIR', HERE.parent / 'maps'))
MAPS_DIR_CONTAINER = '/ros2_ws/src/maps'
WEB_DIR            = HERE / 'web'

ROS_SETUP = 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash'

# name -> (ros2 launch command, substring identifying its process)
LAUNCHES = {
    'stack': ('ros2 launch amr_common amr_common.launch.py', 'amr_common.launch.py'),
    'slam':  ('ros2 launch amr_common slam.launch.py',       'slam.launch.py'),
    'nav':   ('ros2 launch amr_common nav.launch.py map:={map}', 'nav.launch.py'),
}

# Command-line fragments of the long-running nodes each launch starts. Used to
# find (and kill) nodes orphaned when a launch died before its children.
NODE_PATTERNS = {
    'stack': ['esp32_bridge/bridge_node', 'esp32_bridge/odom_node', 'hclidar_driver_ros2_node',
              'rosbridge_websocket', 'rosapi_node', 'foxglove_bridge',
              'static_transform_publisher'],
    'slam':  ['async_slam_toolbox_node'],
    'nav':   ['__node:=nav2_container'],
}

# Which launches each mode needs running (everything else in LAUNCHES is stopped).
MODES = {
    'manual':  {'stack'},
    'mapping': {'stack', 'slam'},
    'auto':    {'stack', 'nav'},
}

# Map names are relative to the maps folder: "my_map", "lab/floor1".
MAP_NAME_RE = re.compile(r'^[A-Za-z0-9_-]+(/[A-Za-z0-9_-]+)*$')

if not API_KEY:
    raise SystemExit('AMR_API_KEY is not set — refusing to start. '
                     'Set it in /etc/amr-supervisor.env')

app = FastAPI(title='AMR Supervisor')

# Serialises mode switches / map saves so two clicks can't interleave
# start/stop sequences.
_op_lock = threading.Lock()


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


def container_ps() -> list:
    """(pid, ppid, args) of every process in the container.

    Runs `ps` directly (no bash -c wrapper), so the query itself never shows
    up as a match -- the old `bash -c "pgrep -f PATTERN"` always matched its
    own wrapper and reported every launch as running.
    """
    r = run(['docker', 'exec', CONTAINER, 'ps', '-eo', 'pid=,ppid=,args='])
    procs = []
    for line in (r.stdout.splitlines() if r.returncode == 0 else []):
        parts = line.split(None, 2)
        if len(parts) == 3:
            procs.append((int(parts[0]), int(parts[1]), parts[2]))
    return procs


def container_processes() -> list:
    """Command lines of every process in the container."""
    return [args for _, _, args in container_ps()]


def launch_running(pattern: str, procs: Optional[list] = None) -> bool:
    procs = container_processes() if procs is None else procs
    return any(pattern in line for line in procs)


def signal_pids(pids, sig: str):
    if pids:
        run(['docker', 'exec', CONTAINER, 'kill', f'-{sig}'] + [str(p) for p in pids])


def stop_launch(name: str, wait_s: float = 20.0):
    """Stop a launch and everything it started.

    SIGINT the `ros2 launch` process (it shuts its nodes down in order), wait,
    then SIGKILL whatever of its process tree is still alive. Only killing the
    launch process -- as this used to do -- orphans its nodes: on the loaded
    Pi Nav2 often needs >8 s to exit, the launch got SIGKILLed first, and its
    nav2_container kept running under PID 1. A second Nav2 with the same node
    names then hijacked /map_server/map, amcl, etc.

    Nodes already orphaned by an earlier stop are found by NODE_PATTERNS.
    """
    pattern = LAUNCHES[name][1] if name in LAUNCHES else name
    node_pats = NODE_PATTERNS.get(name, sum(NODE_PATTERNS.values(), []))
    procs = container_ps()
    children = {}
    for pid, ppid, _ in procs:
        children.setdefault(ppid, []).append(pid)
    tree, todo = set(), [pid for pid, _, a in procs if pattern in a]
    while todo:
        pid = todo.pop()
        if pid not in tree:
            tree.add(pid)
            todo += children.get(pid, [])
    tree |= {pid for pid, _, a in procs if any(p in a for p in node_pats)}
    if not tree:
        return

    # SIGINT the launch processes, plus orphaned nodes (no launch above them).
    # Not the bash -c wrappers: they exit by themselves when their child does.
    by_pid = {pid: (ppid, a) for pid, ppid, a in procs}
    signal_pids([pid for pid in tree
                 if 'ros2 launch' in by_pid[pid][1]
                 or (by_pid[pid][0] not in tree and not by_pid[pid][1].startswith('bash'))],
                'INT')
    deadline = time.time() + wait_s
    while time.time() < deadline:
        time.sleep(0.5)
        alive = tree & {pid for pid, _, _ in container_ps()}
        if not alive:
            return
    signal_pids(sorted(alive), 'KILL')


def kill_orphans(wait_s: float = 10.0) -> list:
    """Stop nodes whose launch died under them (reparented to the container's
    PID 1). Healthy nodes have their `ros2 launch` as parent and are untouched.
    """
    pats = sum(NODE_PATTERNS.values(), [])
    pids = {pid for pid, ppid, a in container_ps()
            if ppid == 1 and any(p in a for p in pats)}
    if not pids:
        return []
    signal_pids(sorted(pids), 'INT')
    deadline = time.time() + wait_s
    alive = pids
    while alive and time.time() < deadline:
        time.sleep(0.5)
        alive = pids & {pid for pid, _, _ in container_ps()}
    signal_pids(sorted(alive), 'KILL')
    return sorted(pids)


def start_launch(name: str, map_path: str = DEFAULT_MAP) -> bool:
    cmd_tpl, pattern = LAUNCHES[name]
    launch_cmd = cmd_tpl.format(map=shlex.quote(map_path))
    log_file = f'{LOG_DIR_CONTAINER}/{name}.log'
    docker_exec(f'mkdir -p {LOG_DIR_CONTAINER}')
    # Fresh log per start so the app's log viewer shows this attempt only.
    docker_exec(f'{ROS_SETUP} && {launch_cmd} > {log_file} 2>&1', detach=True)
    time.sleep(2.0)  # give it a moment to either come up or crash
    return launch_running(pattern)


def running_nav_map(procs: list) -> Optional[str]:
    """Map name the running nav launch was started with, from its cmdline."""
    for line in procs:
        if LAUNCHES['nav'][1] in line and 'map:=' in line:
            path = line.split('map:=', 1)[1].split()[0]
            return map_name_from_container_path(path)
    return None


def map_name_from_container_path(path: str) -> str:
    prefix = MAPS_DIR_CONTAINER + '/'
    if path.startswith(prefix) and path.endswith('.yaml'):
        return path[len(prefix):-len('.yaml')]
    return path


def validate_map_name(name: str) -> str:
    if not name or not MAP_NAME_RE.match(name):
        raise HTTPException(400, 'map name may only contain letters, digits, _ and -, '
                                 'with optional subfolders separated by / (e.g. lab/floor1)')
    return name


def ensure_container():
    cstat = container_status()
    if cstat == 'not_found':
        raise HTTPException(500, f'container "{CONTAINER}" does not exist — '
                                 'create it once with run_ros.sh on the Pi')
    if cstat != 'running':
        r = run(['docker', 'start', CONTAINER], timeout=30)
        if r.returncode != 0:
            raise HTTPException(500, f'docker start failed: {r.stderr.strip()}')


def current_mode(launches: dict) -> str:
    if not launches.get('stack'):
        return 'off'
    if launches.get('nav'):
        return 'auto'
    if launches.get('slam'):
        return 'mapping'
    return 'manual'


class NavRequest(BaseModel):
    map: Optional[str] = None


class ModeRequest(BaseModel):
    mode: str
    map: Optional[str] = None


class SaveMapRequest(BaseModel):
    name: str
    overwrite: bool = False


# ── Web app ────────────────────────────────────────────────────────────────
if WEB_DIR.is_dir():
    app.mount('/app', StaticFiles(directory=WEB_DIR, html=True), name='app')


@app.get('/', include_in_schema=False)
def root():
    return RedirectResponse('/app/')


# ── API ────────────────────────────────────────────────────────────────────
@app.get('/ping')
def ping():
    """Unauthenticated connectivity check."""
    return {'ok': True, 'service': 'amr-supervisor'}


@app.get('/status', dependencies=[Depends(require_key)])
def status():
    cstat = container_status()
    launches = {}
    nav_map = None
    if cstat == 'running':
        procs = container_processes()
        launches = {name: launch_running(pat, procs) for name, (_, pat) in LAUNCHES.items()}
        nav_map = running_nav_map(procs) if launches.get('nav') else None

    temp_c = None
    try:
        temp_c = int(Path('/sys/class/thermal/thermal_zone0/temp').read_text()) / 1000.0
    except (OSError, ValueError):
        pass

    return {
        'container': cstat,
        'launches': launches,
        'mode': current_mode(launches),
        'map': nav_map,
        'busy': _op_lock.locked(),
        'host': {
            'cpu_percent': psutil.cpu_percent(interval=0.2),
            'mem_percent': psutil.virtual_memory().percent,
            'disk_percent': psutil.disk_usage('/').percent,
            'temp_c': temp_c,
        },
    }


@app.post('/system/start', dependencies=[Depends(require_key)])
def system_start():
    ensure_container()
    return {'container': container_status()}


def _system_stop():
    if container_status() == 'running':
        # Graceful: SIGINT any ros2 launch first so nodes shut down cleanly
        # (esp32_bridge zeroes the motors in its shutdown handler).
        stop_launch('ros2 launch')
        r = run(['docker', 'stop', CONTAINER], timeout=40)
        if r.returncode != 0:
            raise HTTPException(500, f'docker stop failed: {r.stderr.strip()}')
    return {'container': container_status()}


@app.post('/system/stop', dependencies=[Depends(require_key)])
def system_stop():
    with _op_lock:
        return _system_stop()


@app.post('/mode', dependencies=[Depends(require_key)])
def set_mode(body: ModeRequest):
    """Bring the stack into one mode: starts what it needs, stops the rest."""
    if body.mode == 'off':
        with _op_lock:
            _system_stop()
        return {'mode': 'off', 'launches': {}}
    if body.mode not in MODES:
        raise HTTPException(400, f'unknown mode "{body.mode}" — use off, {", ".join(MODES)}')

    map_path = None
    if body.mode == 'auto':
        name = validate_map_name(body.map or '')
        if not (MAPS_DIR_HOST / f'{name}.yaml').is_file():
            raise HTTPException(404, f'map "{name}" not found')
        map_path = f'{MAPS_DIR_CONTAINER}/{name}.yaml'

    if not _op_lock.acquire(blocking=False):
        raise HTTPException(409, 'another mode change is in progress')
    try:
        ensure_container()
        wanted = MODES[body.mode]
        killed = kill_orphans()   # duplicates would hijack node names/services
        procs = container_processes()

        # Stop first (frees CPU, and slam + amcl must never both own map->odom).
        for name, (_, pat) in LAUNCHES.items():
            if name not in wanted and launch_running(pat, procs):
                stop_launch(name)
        # Nav already up on a different map -> restart it with the new one.
        if map_path and launch_running(LAUNCHES['nav'][1], procs) \
                and running_nav_map(procs) != body.map:
            stop_launch('nav')

        failed = []
        for name in ('stack', 'slam', 'nav'):  # stack first: everything needs it
            if name not in wanted:
                continue
            if launch_running(LAUNCHES[name][1]):
                continue
            if not start_launch(name, map_path or DEFAULT_MAP):
                failed.append(name)

        procs = container_processes()
        launches = {name: launch_running(pat, procs) for name, (_, pat) in LAUNCHES.items()}
        return {'mode': current_mode(launches), 'launches': launches,
                'map': running_nav_map(procs), 'failed': failed,
                'killed_orphans': killed}
    finally:
        _op_lock.release()


@app.post('/launch/{name}', dependencies=[Depends(require_key)])
def launch_start(name: str, body: Optional[NavRequest] = None):
    if name not in LAUNCHES:
        raise HTTPException(404, f'unknown launch "{name}" — use one of {list(LAUNCHES)}')
    if container_status() != 'running':
        raise HTTPException(409, 'container is not running — call /system/start first')

    _, pattern = LAUNCHES[name]
    if launch_running(pattern):
        return {'launch': name, 'running': True, 'note': 'already running'}

    running = start_launch(name, body.map if body and body.map else DEFAULT_MAP)
    return {'launch': name, 'running': running,
            'log': f'/logs/{name}' if not running else None}


@app.post('/launch/{name}/stop', dependencies=[Depends(require_key)])
def launch_stop(name: str):
    if name not in LAUNCHES:
        raise HTTPException(404, f'unknown launch "{name}" — use one of {list(LAUNCHES)}')
    _, pattern = LAUNCHES[name]
    if container_status() == 'running':
        stop_launch(name)
    return {'launch': name, 'running': launch_running(pattern)
            if container_status() == 'running' else False}


@app.get('/maps', dependencies=[Depends(require_key)])
def list_maps():
    maps = []
    if MAPS_DIR_HOST.is_dir():
        for f in sorted(MAPS_DIR_HOST.rglob('*.yaml')):
            name = f.relative_to(MAPS_DIR_HOST).with_suffix('').as_posix()
            if MAP_NAME_RE.match(name):
                maps.append({'name': name, 'modified': f.stat().st_mtime})
    return {'maps': maps}


@app.post('/maps/save', dependencies=[Depends(require_key)])
def save_map(body: SaveMapRequest):
    name = validate_map_name(body.name)
    yaml_host = MAPS_DIR_HOST / f'{name}.yaml'
    if yaml_host.exists() and not body.overwrite:
        raise HTTPException(409, f'map "{name}" already exists')
    if container_status() != 'running' or not launch_running(LAUNCHES['slam'][1]):
        raise HTTPException(409, 'SLAM is not running — switch to MAPPING mode first')

    with _op_lock:
        out = f'{MAPS_DIR_CONTAINER}/{name}'
        # --free 0.196: unknown cells are written as 205 = occupancy 0.196.
        # With the default free_thresh 0.25 map_server would load them back as
        # FREE; at 0.196 they stay unknown.
        r = docker_exec(
            f'mkdir -p {shlex.quote(os.path.dirname(out))} && {ROS_SETUP} && '
            f'ros2 run nav2_map_server map_saver_cli -f {shlex.quote(out)} '
            f'--occ 0.65 --free 0.196 --ros-args -p save_map_timeout:=10.0',
            timeout=40)
    files = [p.name for p in (yaml_host, yaml_host.with_suffix('.pgm')) if p.exists()]
    ok = r.returncode == 0 and len(files) == 2
    if not ok:
        raise HTTPException(500, 'map_saver_cli failed: ' +
                            (r.stderr.strip() or r.stdout.strip())[-800:])
    return {'name': name, 'files': files}


@app.get('/logs/{name}', dependencies=[Depends(require_key)])
def logs(name: str, lines: int = 100):
    if name not in LAUNCHES:
        raise HTTPException(404, f'unknown launch "{name}" — use one of {list(LAUNCHES)}')
    log_file = LOG_DIR_HOST / f'{name}.log'
    if not log_file.exists():
        return {'launch': name, 'lines': []}
    content = log_file.read_text(errors='ignore').splitlines()
    return {'launch': name, 'lines': content[-lines:]}
