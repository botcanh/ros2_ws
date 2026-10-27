'use strict';
/*
 * AGV control web app.
 *   Supervisor HTTP API (same origin, port 8000): power, modes, maps, logs.
 *   rosbridge websocket (port 9090): drive, telemetry, map, scan, nav.
 */

// ── Config ──────────────────────────────────────────────────────────────────
const ROS_URL      = `ws://${location.hostname || 'localhost'}:9090`;
const LASER_X      = 0.05;   // laser_frame offset from base_link (static_tf.launch.py)
const ROBOT_RADIUS = 0.18;   // robot_radius in nav2_params.yaml
const DRIVE_HZ     = 10;
const CMD_STALE_MS = 600;    // bridge zeroes motors after 0.5 s without cmd_vel
const CHART_SECS   = 15;
const MAP_NAME_RE  = /^[A-Za-z0-9_-]+(\/[A-Za-z0-9_-]+)*$/;
const NAV_STATUS   = {0: 'unknown', 1: 'accepted', 2: 'driving to goal', 3: 'canceling',
                      4: 'goal reached', 5: 'canceled', 6: 'failed (aborted)'};

const $ = id => document.getElementById(id);

// ── Small helpers ───────────────────────────────────────────────────────────
function storeGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
function storeSet(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* ignore */ } }

let toastTimer = null;
function toast(msg, bad = false) {
  const t = $('toast');
  t.textContent = msg;
  t.className = 'toast' + (bad ? ' bad' : '');
  t.hidden = false;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { t.hidden = true; }, bad ? 6000 : 3000);
}

function yawFromQuat(q) {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}
function quatFromYaw(yaw) {
  return {x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2)};
}
function compose(a, b) {  // pose a ∘ pose b (2D)
  const c = Math.cos(a.yaw), s = Math.sin(a.yaw);
  return {x: a.x + c * b.x - s * b.y, y: a.y + s * b.x + c * b.y, yaw: a.yaw + b.yaw};
}
function stripSlash(f) { return (f || '').replace(/^\//, ''); }

// ── Supervisor API ──────────────────────────────────────────────────────────
let apiKey = storeGet('amrKey') || '';

function askKey() {
  const k = prompt('Supervisor API key (AMR_API_KEY in /etc/amr-supervisor.env on the Pi):', apiKey);
  if (k !== null) { apiKey = k.trim(); storeSet('amrKey', apiKey); }
}

async function api(path, body) {
  const opts = {headers: {'X-API-Key': apiKey}};
  if (body !== undefined) {
    opts.method = 'POST';
    opts.headers['Content-Type'] = 'application/json';
    opts.body = JSON.stringify(body);
  }
  const r = await fetch(path, opts);
  let data = null;
  try { data = await r.json(); } catch (e) { /* non-JSON */ }
  if (r.status === 401) {
    askKey();
    throw new Error('API key rejected');
  }
  if (!r.ok) throw new Error((data && data.detail) || `HTTP ${r.status}`);
  return data;
}

// ── App state ───────────────────────────────────────────────────────────────
const S = {
  apiOk: false, rosOk: false,
  mode: 'off',            // actual mode reported by supervisor
  view: 'manual',         // which tab/panel is shown
  busy: false,            // this page is waiting on a mode change
  remoteBusy: false,      // supervisor is mid mode change (maybe from another client)
  loadedMap: null,
  odom: {x: 0, y: 0, yaw: 0}, odomTw: {v: 0, w: 0},
  mapToOdom: null,        // pose of odom in map (from slam_toolbox / amcl TF)
  cmd: {v: 0, w: 0}, cmdStamp: 0,
  scan: null,             // [[x,y], ...] in base_link
  grid: null,             // {canvas, info}
  plan: [],
  goal: null, initPose: null,
  trail: [], trailFrame: null,
  nav: {code: null, text: 'idle'},
  lastMapMsg: 0,
  maps: [],
};

function displayFrame() {
  return (S.mode === 'mapping' || S.mode === 'auto') ? 'map' : 'odom';
}
function robotPose() {
  if (displayFrame() === 'map' && S.mapToOdom) return compose(S.mapToOdom, S.odom);
  return S.odom;
}

// ── rosbridge ───────────────────────────────────────────────────────────────
let ros = null;
let R = {};   // topics/services of the current connection

function connectRos() {
  const conn = new ROSLIB.Ros({url: ROS_URL});
  let closed = false;
  conn.on('connection', () => {
    ros = conn;
    S.rosOk = true;
    setupRos();
    updateLights();
    updateDriveWarn();
  });
  const onDown = () => {
    if (closed) return;
    closed = true;
    if (ros === conn) { ros = null; R = {}; }
    S.rosOk = false;
    updateLights();
    updateDriveWarn();
    setTimeout(connectRos, 2000);
  };
  conn.on('close', onDown);
  conn.on('error', onDown);
}

function topic(name, type, extra = {}) {
  return new ROSLIB.Topic(Object.assign({ros, name, messageType: type}, extra));
}

function setupRos() {
  R.cmdPub   = topic('/cmd_vel', 'geometry_msgs/msg/Twist');
  R.initPub  = topic('/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped');
  R.goalPub  = topic('/goal_pose', 'geometry_msgs/msg/PoseStamped');
  R.cancel   = new ROSLIB.Service({ros, name: '/navigate_to_pose/_action/cancel_goal',
                                   serviceType: 'action_msgs/srv/CancelGoal'});

  topic('/cmd_vel', 'geometry_msgs/msg/Twist', {throttle_rate: 100, queue_length: 1})
    .subscribe(m => { S.cmd = {v: m.linear.x, w: m.angular.z}; S.cmdStamp = Date.now(); });

  topic('/odom', 'nav_msgs/msg/Odometry', {throttle_rate: 100, queue_length: 1})
    .subscribe(m => {
      const p = m.pose.pose;
      S.odom = {x: p.position.x, y: p.position.y, yaw: yawFromQuat(p.orientation)};
      S.odomTw = {v: m.twist.twist.linear.x, w: m.twist.twist.angular.z};
    });

  // Not throttled: map->odom and odom->base_link arrive in separate messages,
  // throttling could starve one of them.
  topic('/tf', 'tf2_msgs/msg/TFMessage').subscribe(m => {
    for (const t of m.transforms) {
      if (stripSlash(t.header.frame_id) === 'map' && stripSlash(t.child_frame_id) === 'odom') {
        const tr = t.transform;
        S.mapToOdom = {x: tr.translation.x, y: tr.translation.y, yaw: yawFromQuat(tr.rotation)};
      }
    }
  });

  topic('/scan', 'sensor_msgs/msg/LaserScan', {throttle_rate: 200, queue_length: 1, compression: 'cbor'})
    .subscribe(m => {
      const pts = [];
      const n = m.ranges.length;
      for (let i = 0; i < n; i++) {
        const r = m.ranges[i];
        if (r === null || !isFinite(r) || r < m.range_min || r > m.range_max) continue;
        const a = m.angle_min + i * m.angle_increment;
        pts.push([LASER_X + r * Math.cos(a), r * Math.sin(a)]);
      }
      S.scan = pts;
    });

  topic('/plan', 'nav_msgs/msg/Path', {throttle_rate: 500, queue_length: 1})
    .subscribe(m => { S.plan = m.poses.map(p => [p.pose.position.x, p.pose.position.y]); });

  topic('/navigate_to_pose/_action/status', 'action_msgs/msg/GoalStatusArray')
    .subscribe(m => {
      const list = m.status_list || [];
      if (!list.length) return;
      const code = list[list.length - 1].status;
      S.nav = {code, text: NAV_STATUS[code] || String(code)};
      if (code >= 4) { S.plan = []; if (code === 4) S.goal = null; }
      $('navStatus').textContent = S.nav.text;
    });

  subscribeMap();
  fetchMapWithRetry();
}

// /map is re-subscribed on every mode change: rosbridge picks the QoS from the
// publishers present at subscribe time, and map_server's map is latched.
function subscribeMap() {
  if (!ros) return;
  if (R.mapSub) R.mapSub.unsubscribe();
  R.mapSub = topic('/map', 'nav_msgs/msg/OccupancyGrid', {queue_length: 1, compression: 'cbor'});
  R.mapSub.subscribe(onGrid);
}

let mapFetchToken = 0;
function fetchMapWithRetry() {
  const token = ++mapFetchToken;
  const attempt = (n) => {
    if (token !== mapFetchToken || !ros) return;
    const name = S.mode === 'auto' ? '/map_server/map'
               : S.mode === 'mapping' ? '/slam_toolbox/dynamic_map' : null;
    if (!name) return;
    const svc = new ROSLIB.Service({ros, name, serviceType: 'nav_msgs/srv/GetMap'});
    let done = false;
    const retry = () => { if (!done) { done = true; if (n < 30) setTimeout(() => attempt(n + 1), 3000); } };
    svc.callService(new ROSLIB.ServiceRequest({}), res => {
      done = true;
      if (res && res.map && res.map.info && res.map.info.width > 0) onGrid(res.map);
      else if (n < 30) setTimeout(() => attempt(n + 1), 3000);
    }, retry);
    setTimeout(retry, 5000);  // service not up yet -> call may never answer
  };
  attempt(0);
}

function onGrid(m) {
  const {width: w, height: h} = m.info;
  if (!w || !h) return;
  const c = document.createElement('canvas');
  c.width = w; c.height = h;
  const ctx = c.getContext('2d');
  const img = ctx.createImageData(w, h);
  const d = m.data;
  for (let row = 0; row < h; row++) {
    const out = (h - 1 - row) * w;     // grid row 0 is the bottom (lowest y)
    for (let col = 0; col < w; col++) {
      const v = d[row * w + col];
      const o = (out + col) * 4;
      if (v < 0) { img.data[o] = 42; img.data[o + 1] = 53; img.data[o + 2] = 64; }
      else {
        const k = 1 - Math.min(v, 100) / 100;   // 1 free .. 0 occupied
        img.data[o] = 12 + 213 * k; img.data[o + 1] = 14 + 217 * k; img.data[o + 2] = 18 + 218 * k;
      }
      img.data[o + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);
  const first = !S.grid;
  S.grid = {canvas: c, info: m.info};
  S.lastMapMsg = Date.now();
  if (first) fitView();
}

// ── Commands to ROS ─────────────────────────────────────────────────────────
function publishTwist(v, w) {
  if (!R.cmdPub) return;
  R.cmdPub.publish({linear: {x: v, y: 0, z: 0}, angular: {x: 0, y: 0, z: w}});
}

function navActive() { return S.nav.code === 1 || S.nav.code === 2; }

function cancelGoal(quiet = false) {
  if (!R.cancel) return;
  R.cancel.callService(new ROSLIB.ServiceRequest({
    goal_info: {goal_id: {uuid: new Array(16).fill(0)}, stamp: {sec: 0, nanosec: 0}},
  }), () => { if (!quiet) toast('Goal canceled'); }, () => { /* nav not running */ });
  S.goal = null;
  S.plan = [];
}

function publishInitPose(x, y, yaw) {
  const cov = new Array(36).fill(0);
  cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.0685;
  R.initPub.publish({
    header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}},
    pose: {pose: {position: {x, y, z: 0}, orientation: quatFromYaw(yaw)}, covariance: cov},
  });
  S.initPose = {x, y, yaw, t: Date.now()};
  toast('Initial pose sent');
}

function publishGoal(x, y, yaw) {
  R.goalPub.publish({
    header: {frame_id: 'map', stamp: {sec: 0, nanosec: 0}},
    pose: {position: {x, y, z: 0}, orientation: quatFromYaw(yaw)},
  });
  S.goal = {x, y, yaw};
  toast('Goal sent');
}

// ── Drive (joystick + keyboard) ─────────────────────────────────────────────
const joy = {active: false, x: 0, y: 0};
const keys = new Set();
let wasDriving = false;
let stopBurst = 0;

function driveVector() {
  if (joy.active) return {fx: joy.y, fw: -joy.x};
  const f = (keys.has('w') || keys.has('arrowup') ? 1 : 0) - (keys.has('s') || keys.has('arrowdown') ? 1 : 0);
  const t = (keys.has('a') || keys.has('arrowleft') ? 1 : 0) - (keys.has('d') || keys.has('arrowright') ? 1 : 0);
  return {fx: f, fw: t};
}

function driveBlocker() {
  if (S.mode === 'off') return 'AGV is OFF — press Power ON (top bar) first.';
  if (!R.cmdPub) return 'Not connected to rosbridge (ROS light is red) — wait a few seconds after Power ON.';
  return null;
}

function updateDriveWarn() {
  const msg = driveBlocker();
  $('driveWarn').textContent = msg || '';
  $('driveWarn').hidden = !msg;
  $('joy').classList.toggle('blocked', !!msg);
}

function driveTick() {
  const {fx, fw} = driveVector();
  const driving = joy.active || keys.size > 0;
  if (driving && !wasDriving && driveBlocker()) toast(driveBlocker(), true);
  if (driving) {
    if (!wasDriving && S.mode === 'auto' && navActive()) {
      cancelGoal(true);
      toast('Manual drive: goal canceled');
    }
    publishTwist(fx * parseFloat($('maxV').value), fw * parseFloat($('maxW').value));
    stopBurst = 3;
  } else if (stopBurst > 0) {
    publishTwist(0, 0);
    stopBurst--;
  }
  wasDriving = driving;
}

function setupJoystick() {
  const pad = $('joy'), knob = $('joyKnob');
  const R0 = 70, KR = 24;
  const move = (e) => {
    const r = pad.getBoundingClientRect();
    let dx = e.clientX - r.left - R0, dy = e.clientY - r.top - R0;
    const lim = R0 - KR, len = Math.hypot(dx, dy);
    if (len > lim) { dx *= lim / len; dy *= lim / len; }
    knob.style.left = (R0 - KR + dx) + 'px';
    knob.style.top  = (R0 - KR + dy) + 'px';
    joy.x = dx / lim; joy.y = -dy / lim;
  };
  const end = () => {
    joy.active = false; joy.x = joy.y = 0;
    knob.style.left = knob.style.top = (R0 - KR) + 'px';
    pad.classList.remove('active');
  };
  pad.addEventListener('pointerdown', e => {
    joy.active = true; pad.classList.add('active');
    pad.setPointerCapture(e.pointerId); move(e);
  });
  pad.addEventListener('pointermove', e => { if (joy.active) move(e); });
  pad.addEventListener('pointerup', end);
  pad.addEventListener('pointercancel', end);
  joy.release = end;
}

const DRIVE_KEYS = ['w', 'a', 's', 'd', 'arrowup', 'arrowdown', 'arrowleft', 'arrowright'];
function setupKeys() {
  window.addEventListener('keydown', e => {
    const tag = (e.target.tagName || '').toLowerCase();
    if (tag === 'input' || tag === 'select' || tag === 'textarea') return;
    const k = e.key.toLowerCase();
    if (k === ' ') { e.preventDefault(); keys.clear(); publishTwist(0, 0); return; }
    if (k === 'escape') { setTool(null); return; }
    if (DRIVE_KEYS.includes(k)) { e.preventDefault(); keys.add(k); }
  });
  window.addEventListener('keyup', e => keys.delete(e.key.toLowerCase()));
  window.addEventListener('blur', () => keys.clear());
}

function estop() {
  keys.clear();
  if (joy.release) joy.release();
  cancelGoal(true);
  let n = 0;
  const burst = setInterval(() => { publishTwist(0, 0); if (++n >= 10) clearInterval(burst); }, 100);
  toast('E-STOP: goal canceled, zero velocity sent', true);
}

// ── Map view ────────────────────────────────────────────────────────────────
const canvas = $('map');
const ctx = canvas.getContext('2d');
const V = {cx: 0, cy: 0, s: 80};   // view centre (m) and scale (px/m)
let tool = null, toolStart = null, toolEnd = null, pan = null;

function toScreen(x, y) {
  return [(x - V.cx) * V.s + canvas.clientWidth / 2, canvas.clientHeight / 2 - (y - V.cy) * V.s];
}
function toWorld(px, py) {
  return [(px - canvas.clientWidth / 2) / V.s + V.cx, V.cy - (py - canvas.clientHeight / 2) / V.s];
}

function fitView() {
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (S.grid && displayFrame() === 'map') {
    const i = S.grid.info, mw = i.width * i.resolution, mh = i.height * i.resolution;
    V.cx = i.origin.position.x + mw / 2;
    V.cy = i.origin.position.y + mh / 2;
    V.s = Math.max(5, Math.min(W / mw, H / mh) * 0.92);
  } else {
    const p = robotPose();
    V.cx = p.x; V.cy = p.y; V.s = 100;
  }
}

function setTool(t) {
  tool = t; toolStart = toolEnd = null;
  document.querySelectorAll('.tool').forEach(b => b.classList.toggle('active', b.dataset.tool === t));
  canvas.classList.toggle('tooling', !!t);
  $('toolHint').textContent = t ? 'Click the position, drag toward the heading, release. Esc cancels.' : '';
}

function setupMapInput() {
  canvas.addEventListener('pointerdown', e => {
    canvas.setPointerCapture(e.pointerId);
    const r = canvas.getBoundingClientRect();
    const px = e.clientX - r.left, py = e.clientY - r.top;
    if (tool) { toolStart = toWorld(px, py); toolEnd = toolStart; }
    else pan = {px, py, cx: V.cx, cy: V.cy};
  });
  canvas.addEventListener('pointermove', e => {
    const r = canvas.getBoundingClientRect();
    const px = e.clientX - r.left, py = e.clientY - r.top;
    if (tool && toolStart) toolEnd = toWorld(px, py);
    else if (pan) {
      V.cx = pan.cx - (px - pan.px) / V.s;
      V.cy = pan.cy + (py - pan.py) / V.s;
      $('chkFollow').checked = false;
    }
  });
  const up = () => {
    if (tool && toolStart) {
      const [x, y] = toolStart, [ex, ey] = toolEnd;
      const yaw = Math.hypot(ex - x, ey - y) * V.s > 8 ? Math.atan2(ey - y, ex - x) : robotPose().yaw;
      if (!ros) toast('Not connected to rosbridge', true);
      else if (tool === 'initpose') publishInitPose(x, y, yaw);
      else publishGoal(x, y, yaw);
      setTool(null);
    }
    pan = null;
  };
  canvas.addEventListener('pointerup', up);
  canvas.addEventListener('pointercancel', up);
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const r = canvas.getBoundingClientRect();
    const px = e.clientX - r.left, py = e.clientY - r.top;
    const [wx, wy] = toWorld(px, py);
    V.s = Math.max(5, Math.min(800, V.s * Math.exp(-e.deltaY * 0.0015)));
    // keep the point under the cursor fixed
    V.cx = wx - (px - canvas.clientWidth / 2) / V.s;
    V.cy = wy + (py - canvas.clientHeight / 2) / V.s;
  }, {passive: false});
}

function drawArrow(x, y, yaw, color, len = 0.35) {
  const [sx, sy] = toScreen(x, y);
  const [ex, ey] = toScreen(x + len * Math.cos(yaw), y + len * Math.sin(yaw));
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke();
  const a = Math.atan2(ey - sy, ex - sx);
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - 10 * Math.cos(a - 0.45), ey - 10 * Math.sin(a - 0.45));
  ctx.lineTo(ex - 10 * Math.cos(a + 0.45), ey - 10 * Math.sin(a + 0.45));
  ctx.closePath(); ctx.fill();
  ctx.beginPath(); ctx.arc(sx, sy, 4, 0, 2 * Math.PI); ctx.fill();
}

function drawGridLines() {
  const W = canvas.clientWidth, H = canvas.clientHeight;
  const step = V.s > 150 ? 0.5 : V.s > 30 ? 1 : 5;
  const [x0, y1] = toWorld(0, 0), [x1, y0] = toWorld(W, H);
  ctx.lineWidth = 1;
  for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) {
    const [sx] = toScreen(x, 0);
    ctx.strokeStyle = Math.abs(x) < 1e-6 ? '#3a4a59' : '#1a232c';
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, H); ctx.stroke();
  }
  for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) {
    const [, sy] = toScreen(0, y);
    ctx.strokeStyle = Math.abs(y) < 1e-6 ? '#3a4a59' : '#1a232c';
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(W, sy); ctx.stroke();
  }
}

function render() {
  const dpr = window.devicePixelRatio || 1;
  const W = canvas.clientWidth, H = canvas.clientHeight;
  if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
    canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.fillStyle = '#0b0f13';
  ctx.fillRect(0, 0, W, H);

  const frame = displayFrame();
  const pose = robotPose();
  if ($('chkFollow').checked) { V.cx = pose.x; V.cy = pose.y; }

  if (frame === 'map' && S.grid) {
    const i = S.grid.info;
    const [sx, sy] = toScreen(i.origin.position.x, i.origin.position.y + i.height * i.resolution);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(S.grid.canvas, sx, sy, i.width * i.resolution * V.s, i.height * i.resolution * V.s);
  } else {
    drawGridLines();
  }

  // trail (reset when the display frame changes)
  if (S.trailFrame !== frame) { S.trail = []; S.trailFrame = frame; }
  const last = S.trail[S.trail.length - 1];
  if (!last || Math.hypot(last[0] - pose.x, last[1] - pose.y) > 0.03) {
    S.trail.push([pose.x, pose.y]);
    if (S.trail.length > 2000) S.trail.shift();
  }
  if (S.trail.length > 1) {
    ctx.strokeStyle = '#3fa7ff66'; ctx.lineWidth = 2; ctx.beginPath();
    S.trail.forEach(([x, y], k) => { const [a, b] = toScreen(x, y); k ? ctx.lineTo(a, b) : ctx.moveTo(a, b); });
    ctx.stroke();
  }

  // planned path
  if (frame === 'map' && S.plan.length > 1) {
    ctx.strokeStyle = '#3ccf7a'; ctx.lineWidth = 2.5; ctx.beginPath();
    S.plan.forEach(([x, y], k) => { const [a, b] = toScreen(x, y); k ? ctx.lineTo(a, b) : ctx.moveTo(a, b); });
    ctx.stroke();
  }

  // laser scan
  if (S.scan) {
    ctx.fillStyle = '#ff5b5b';
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    for (const [bx, by] of S.scan) {
      const [a, b] = toScreen(pose.x + c * bx - s * by, pose.y + s * bx + c * by);
      ctx.fillRect(a - 1.5, b - 1.5, 3, 3);
    }
  }

  // goal / initial pose markers
  if (frame === 'map' && S.goal) drawArrow(S.goal.x, S.goal.y, S.goal.yaw, '#3ccf7a', 0.4);
  if (frame === 'map' && S.initPose && Date.now() - S.initPose.t < 4000)
    drawArrow(S.initPose.x, S.initPose.y, S.initPose.yaw, '#f2b33d', 0.4);

  // robot
  const [rx, ry] = toScreen(pose.x, pose.y);
  ctx.fillStyle = '#3fa7ff33'; ctx.strokeStyle = '#3fa7ff'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(rx, ry, Math.max(6, ROBOT_RADIUS * V.s), 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
  drawArrow(pose.x, pose.y, pose.yaw, '#3fa7ff', Math.max(ROBOT_RADIUS * 1.3, 14 / V.s));

  // tool preview
  if (tool && toolStart) {
    const [x, y] = toolStart, [ex, ey] = toolEnd;
    const yaw = Math.hypot(ex - x, ey - y) * V.s > 8 ? Math.atan2(ey - y, ex - x) : pose.yaw;
    drawArrow(x, y, yaw, tool === 'goal' ? '#3ccf7a' : '#f2b33d', 0.4);
  }

  let label = `frame: ${frame}`;
  if (frame === 'map' && !S.mapToOdom) label += ' (no map→odom TF yet — showing odom)';
  if (frame === 'map' && !S.grid) label += ' · waiting for map…';
  $('frameLabel').textContent = label;

  requestAnimationFrame(render);
}

// ── Telemetry + charts ──────────────────────────────────────────────────────
const hist = [];
function telemetryTick() {
  const stale = Date.now() - S.cmdStamp > CMD_STALE_MS;
  const cv = stale ? 0 : S.cmd.v, cw = stale ? 0 : S.cmd.w;
  $('cmdV').textContent = cv.toFixed(2);
  $('cmdW').textContent = cw.toFixed(2);
  $('odX').textContent = S.odom.x.toFixed(2);
  $('odY').textContent = S.odom.y.toFixed(2);
  $('odYaw').textContent = (S.odom.yaw * 180 / Math.PI).toFixed(1);
  $('odV').textContent = S.odomTw.v.toFixed(2);
  $('odW').textContent = S.odomTw.w.toFixed(2);

  hist.push({cv, cw, ov: S.odomTw.v, ow: S.odomTw.w});
  if (hist.length > CHART_SECS * 10) hist.shift();
  drawChart($('chartV'), 'cv', 'ov', 0.25, 'v m/s');
  drawChart($('chartW'), 'cw', 'ow', 1.0, 'ω rad/s');

  // Nav2's planner only finishes starting once AMCL has a pose (map->odom TF).
  if (S.mode === 'auto' && S.nav.code === null) {
    $('navStatus').textContent = !S.grid ? 'loading map… (Nav2 starting)'
      : !S.mapToOdom ? 'set initial pose to finish starting Nav2' : 'ready — set a goal';
  }

  // MAPPING: if /map updates stop arriving, fall back to the service.
  if (S.mode === 'mapping' && ros && Date.now() - S.lastMapMsg > 12000) {
    S.lastMapMsg = Date.now();
    fetchMapWithRetry();
  }
}

function drawChart(cv, a, b, minRange, label) {
  const dpr = window.devicePixelRatio || 1;
  const W = cv.clientWidth, H = cv.clientHeight;
  if (cv.width !== Math.round(W * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
  const c = cv.getContext('2d');
  c.setTransform(dpr, 0, 0, dpr, 0, 0);
  c.clearRect(0, 0, W, H);
  let range = minRange;
  for (const h of hist) range = Math.max(range, Math.abs(h[a]), Math.abs(h[b]));
  const y = v => H / 2 - (v / range) * (H / 2 - 4);
  c.strokeStyle = '#2c3a47'; c.lineWidth = 1;
  c.beginPath(); c.moveTo(0, H / 2); c.lineTo(W, H / 2); c.stroke();
  const n = CHART_SECS * 10;
  for (const [key, color] of [[a, '#3fa7ff'], [b, '#f2b33d']]) {
    c.strokeStyle = color; c.lineWidth = 1.5; c.beginPath();
    hist.forEach((h, k) => {
      const x = W - (hist.length - 1 - k) * (W / n);
      k ? c.lineTo(x, y(h[key])) : c.moveTo(x, y(h[key]));
    });
    c.stroke();
  }
  c.fillStyle = '#8a9aa8'; c.font = '11px system-ui';
  c.fillText(`${label}  ±${range.toFixed(2)}`, 6, 12);
}

// ── Status / modes / maps ───────────────────────────────────────────────────
function updateLights() {
  $('lightApi').className = 'light ' + (S.apiOk ? 'ok' : 'bad');
  $('lightRos').className = 'light ' + (S.rosOk ? 'ok' : (S.mode === 'off' ? '' : 'bad'));
}

function applyMode(mode, map) {
  const changed = mode !== S.mode || (mode === 'auto' && map !== S.loadedMap);
  S.mode = mode;
  S.loadedMap = map;
  if (!changed) return;
  S.view = mode === 'off' ? S.view : mode;
  S.grid = null; S.plan = []; S.goal = null; S.mapToOdom = null;
  S.nav = {code: null, text: 'idle'};
  $('navStatus').textContent = 'idle';
  subscribeMap();
  fetchMapWithRetry();
  if (mode === 'manual' || mode === 'off') fitView();
  renderPanels();
  refreshMaps();
}

function renderPanels() {
  const busy = S.busy || S.remoteBusy;
  document.querySelectorAll('#modeTabs button').forEach(b => {
    // No tab lit while powered off -- a lit MANUAL tab looked like "driving works".
    b.classList.toggle('active', S.mode !== 'off' && b.dataset.mode === S.view);
    b.disabled = busy;
  });
  updateDriveWarn();
  $('modeLabel').textContent = S.mode.toUpperCase() + (S.mode === 'auto' && S.loadedMap ? ` · ${S.loadedMap}` : '');
  $('busy').hidden = !busy;
  $('panelMapping').hidden = S.view !== 'mapping';
  $('panelAuto').hidden = S.view !== 'auto';
  const auto = S.mode === 'auto';
  $('btnInitPose').hidden = !auto;
  $('btnGoal').hidden = !auto;
  if (!auto && tool) setTool(null);
  $('btnPowerOn').disabled = busy || S.mode !== 'off';
  $('btnPowerOff').disabled = busy || S.mode === 'off';
  $('btnLoadMap').disabled = busy;
  $('btnSaveMap').disabled = busy || S.mode !== 'mapping';
  $('loadedMap').textContent = S.loadedMap || '—';
}

async function pollStatus() {
  try {
    const st = await api('/status');
    S.apiOk = true;
    S.remoteBusy = !!st.busy;
    applyMode(st.mode, st.map);
    const h = st.host || {};
    $('hostInfo').textContent = `container: ${st.container} · CPU ${Math.round(h.cpu_percent ?? 0)}%`
      + (h.temp_c != null ? ` · ${h.temp_c.toFixed(0)}°C` : '');
  } catch (e) {
    S.apiOk = false;
    $('hostInfo').textContent = 'supervisor unreachable: ' + e.message;
  }
  updateLights();
  renderPanels();
}

async function switchMode(mode, map) {
  if (S.busy) return;
  S.busy = true; renderPanels();
  toast(`Switching to ${mode.toUpperCase()}… (Nav2 can take ~20 s)`);
  try {
    const r = await api('/mode', {mode, map});
    if (r.failed && r.failed.length) {
      toast(`Failed to start: ${r.failed.join(', ')} — check Logs`, true);
      $('logSelect').value = r.failed[0];
      showLogs();
    } else {
      toast(`Mode: ${r.mode.toUpperCase()}`);
    }
  } catch (e) {
    toast('Mode change failed: ' + e.message, true);
  }
  S.busy = false;
  await pollStatus();
}

async function refreshMaps() {
  try {
    const r = await api('/maps');
    S.maps = r.maps;
    const sel = $('mapSelect'), keep = sel.value || S.loadedMap;
    sel.innerHTML = '';
    for (const m of r.maps) {
      const o = document.createElement('option');
      o.value = o.textContent = m.name;
      sel.appendChild(o);
    }
    if (keep && r.maps.some(m => m.name === keep)) sel.value = keep;
    const ul = $('mapListMapping');
    ul.innerHTML = '';
    for (const m of r.maps) {
      const li = document.createElement('li');
      li.textContent = `${m.name}  (${new Date(m.modified * 1000).toLocaleString()})`;
      ul.appendChild(li);
    }
  } catch (e) { /* shown via status */ }
}

async function saveMap() {
  const name = $('mapName').value.trim();
  const res = $('saveResult');
  if (!MAP_NAME_RE.test(name)) {
    res.className = 'result bad';
    res.textContent = 'Use letters, digits, _ and -, optional subfolders with / (e.g. lab/floor1).';
    return;
  }
  let overwrite = false;
  if (S.maps.some(m => m.name === name)) {
    if (!confirm(`Map "${name}" already exists. Overwrite it?`)) return;
    overwrite = true;
  }
  res.className = 'result'; res.textContent = 'Saving…';
  $('btnSaveMap').disabled = true;
  try {
    const r = await api('/maps/save', {name, overwrite});
    res.className = 'result ok';
    res.textContent = `Saved ~/ros2_ws/src/maps/${r.name}.yaml + .pgm`;
    refreshMaps();
  } catch (e) {
    res.className = 'result bad';
    res.textContent = 'Save failed: ' + e.message;
  }
  renderPanels();
}

async function showLogs() {
  const name = $('logSelect').value;
  const pre = $('logView');
  pre.hidden = false;
  try {
    const r = await api(`/logs/${name}?lines=200`);
    pre.textContent = r.lines.length ? r.lines.join('\n') : `(no ${name}.log yet)`;
    pre.scrollTop = pre.scrollHeight;
  } catch (e) { pre.textContent = 'Error: ' + e.message; }
}

// ── Wire up UI ──────────────────────────────────────────────────────────────
function setupUi() {
  $('btnPowerOn').onclick = () => switchMode('manual');
  $('btnPowerOff').onclick = () => {
    if (confirm('Power OFF: stop all ROS nodes and the container?')) switchMode('off');
  };
  document.querySelectorAll('#modeTabs button').forEach(b => {
    b.onclick = () => {
      const m = b.dataset.mode;
      if (m === 'auto') {            // pick a map first, then "Load"
        S.view = 'auto'; refreshMaps(); renderPanels();
        return;
      }
      if (m === S.mode) { S.view = m; renderPanels(); return; }
      if (S.mode === 'mapping' && !confirm('Leave MAPPING? The current map is lost unless you saved it.')) return;
      S.view = m;
      if (m === 'mapping') refreshMaps();
      switchMode(m);
    };
  });
  $('btnLoadMap').onclick = () => {
    const map = $('mapSelect').value;
    if (!map) { toast('No saved maps yet — make one in MAPPING mode', true); return; }
    if (S.mode === 'mapping' && !confirm('Leave MAPPING? The current map is lost unless you saved it.')) return;
    switchMode('auto', map);
  };
  $('btnCancel').onclick = () => cancelGoal();
  $('btnEstop').onclick = estop;
  $('btnSaveMap').onclick = saveMap;
  $('btnLogs').onclick = showLogs;
  $('btnKey').onclick = askKey;
  $('btnFit').onclick = fitView;
  document.querySelectorAll('.tool').forEach(b => {
    b.onclick = () => setTool(tool === b.dataset.tool ? null : b.dataset.tool);
  });
  const lbl = () => {
    $('maxVLabel').textContent = parseFloat($('maxV').value).toFixed(2);
    $('maxWLabel').textContent = parseFloat($('maxW').value).toFixed(2);
  };
  $('maxV').oninput = $('maxW').oninput = lbl;
  lbl();
}

function main() {
  // http://<pi>:8000/?key=XXXX stores the key once, then drops it from the URL.
  const urlKey = new URLSearchParams(location.search).get('key');
  if (urlKey) {
    apiKey = urlKey; storeSet('amrKey', apiKey);
    history.replaceState(null, '', location.pathname);
  }
  if (!apiKey) askKey();
  setupUi();
  setupJoystick();
  setupKeys();
  setupMapInput();
  renderPanels();
  updateLights();
  fitView();
  requestAnimationFrame(render);
  setInterval(driveTick, 1000 / DRIVE_HZ);
  setInterval(telemetryTick, 100);
  setInterval(pollStatus, 2000);
  pollStatus();
  refreshMaps();
  connectRos();
}

main();
