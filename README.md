# Fusion TCP server

Runs the Raedir EKF fusion stack via `libfusion.so` and forwards fused poses to the pose viewer (Vercel or localhost).

## Local pose viewer (replace Vercel)

Run the viewer on your PC and point webhooks to localhost instead of Vercel.

### Pose viewer (`vercel_pose_viewer`)

```powershell
cd C:\Users\carno\Documents\GitHub\vercel_pose_viewer
copy .env.example .env.local
npm install
npm run dev
```

Open **http://localhost:3000**. Set `USE_LOCAL_STORE=true` and `WEBHOOK_SECRET` in `.env.local` (see that repo’s README).

### Fusion server config

In `config.json` or `fusion_server.json`:

```json
"VERCEL_WEBHOOK_URL": "http://127.0.0.1:3000/api/gadget",
"WEBHOOK_SECRET": "my_webhook_secret_321"
```

Secrets must match the viewer’s `.env.local`.

### Run order

1. `npm run dev` — pose viewer on port 3000  
2. `python fusion_server.py` — fusion TCP server  
3. Collar connects to port 9000  
4. Admin console: `display start`  
5. Cube updates at http://localhost:3000  

If fusion runs on **another machine** (e.g. Oracle) but the viewer is on your PC, run `npm run dev:local` in the viewer repo and set `VERCEL_WEBHOOK_URL` to `http://<your-pc-lan-ip>:3000/api/gadget` on the remote server.

---

## Setup on Oracle Ubuntu

```bash
cd services/fusion_server
chmod +x build_lib.sh
./build_lib.sh

export SERVER_HOST=0.0.0.0
export SERVER_PORT=9000
export VERCEL_WEBHOOK_URL="https://your-app.vercel.app/api/gadget"
export WEBHOOK_SECRET="your-shared-secret"
export STREAM_IDLE_TIMEOUT_S=3

python3 fusion_server.py
```

Open port 9000 in Oracle Cloud security list / firewall:

```bash
sudo ufw allow 9000/tcp
```

## Live collar streaming → Vercel

The collar connects to the server and **streams packets permanently**. All control
(calibration, live website display) is from the **server admin console**.

```
Collar → TCP :9000 → fusion_server → (display start) → Vercel webhook → website
```

### 1. Start the server (Oracle)

```bash
cd ~/fusion_server
git pull && ./build_lib.sh

# /etc/fusion-server/env
SERVER_HOST=0.0.0.0
SERVER_PORT=9000
ADMIN_HOST=127.0.0.1
ADMIN_PORT=9001
VERCEL_WEBHOOK_URL=https://your-app.vercel.app/api/gadget
WEBHOOK_SECRET=your-shared-secret
STREAM_LATENCY_S=auto

python3 fusion_server.py
```

Open TCP **9000** (collar stream) in Oracle firewall. Admin port **9001** stays localhost-only.

### 2. Collar streams packets (no control messages)

The collar only sends sensor lines, e.g.:

```json
[0, 1234, [x, y, z, w]]
[2, 1235, [dx, dy, 255]]
[3, 1236, [550]]
```

| Type | Data |
|------|------|
| 0 | Accel `[x,y,z]` or quat `[x,y,z,w]` if 4 values |
| 1 | Quaternion `[x,y,z,w]` |
| 2 | Flow `[dx, dy, quality]` |
| 3 | Radar `[mm]` |

### 3. Server admin commands

**Interactive** (when server runs in foreground with TTY):

```
fusion> help
fusion> status
fusion> cal start
fusion> cal finish
fusion> display start
fusion> display stop
fusion> record imu
fusion> record stop
```

**From another SSH session** (systemd / background):

```bash
cd ~/fusion_server
python3 fusion_admin.py status
python3 fusion_admin.py cal start
# spin collar on textured surface for 5+ seconds
python3 fusion_admin.py cal finish
python3 fusion_admin.py display start
python3 fusion_admin.py display stop
```

**Record IMU data for offline lever-arm testing** (no flow/radar in the file):

```bash
python3 fusion_admin.py "record imu"
# spin collar steadily about one axis for several seconds
python3 fusion_admin.py "record stop"
```

Default output: `recordings/cal_imu_YYYYMMDD_HHMMSS.jsonl`. Optional path:
`record imu my_spin.jsonl`.

**File format: use `.jsonl`, not `.json`**

| Extension | Use |
|-----------|-----|
| **`.jsonl`** | Sensor captures — one JSON value per line (metadata line, then wire batches). Used by `record`, `replay`, and `calibrate_capture_file()`. |
| **`.json`** | Single JSON document only — e.g. `fusion_calib.json` (calibration output). Do **not** save multi-minute sensor streams as one `.json` array; tools expect JSONL. |

If you omit an extension on `record imu myfile`, the server appends `.jsonl` automatically.

One-shot without REPL:

```bash
python3 fusion_admin.py cal start
```

### 4. Calibration procedure

1. Collar connected and streaming (`status` shows connected).
2. Place collar flat on textured surface.
3. `cal start`
4. Spin collar about **one axis** through its center for 5+ seconds.
5. `cal finish` — saves `fusion_calib.json` on the server.
6. `cal status` to inspect progress any time.

### 5. Live website display

1. `display start` — fusion runs, poses POST to Vercel.
2. Move collar on the table; watch the website.
3. `display stop` — stops webhook updates (collar keeps streaming).

### 6. Optional USB serial bridge

If the collar only has USB serial, use `collar_stream.py` as a dumb forwarder
(sensor lines only — no calibration logic on the PC):

```bash
python3 collar_stream.py --serial /dev/ttyACM0 --host 127.0.0.1
```

(On the same machine as the server, forward to localhost:9000.)

## Protocol (newline-delimited JSON)

| Message | Purpose |
|---------|---------|
| `{"type":"start"}` | Reset filter, begin streaming session |
| `{"type":"sensor", ...}` | Submit IMU / flow / range sample |
| `{"type":"end"}` | End session; webhook gets `streaming: false` |
| `{"type":"cal_lever_arm_start","axis":"auto","omega_rad_s":0}` | Begin calibration; `auto` detects spin axis |
| `{"type":"cal_lever_arm_finish"}` | Compute, apply, and save lever arm |
| `{"type":"cal_lever_arm_cancel"}` | Abort calibration |
| `{"type":"cal_lever_arm_status"}` | Query calibration progress / current arm |
| `{"type":"stream_status"}` | Query adaptive buffer latency and sensor periods |

Sensor fields (all optional per line, but send a full set each tick for best fusion):

```json
{
  "type": "sensor",
  "ts_us": 1234567890123,
  "quat": {"w": 1, "x": 0, "y": 0, "z": 0},
  "gyro": {"x": 0, "y": 0, "z": 0},
  "accel": {"x": 0, "y": 0, "z": 0},
  "flow": {"dx": 0, "dy": -5, "quality": 255},
  "range": {"mm": 550}
}
```

Server replies with ack lines:

```json
{"type":"ack","of":"start","session_id":"..."}
```

## Test locally

```bash
./build_lib.sh
python3 fusion_server.py &
python3 client_example.py
```

## Environment variables

| Variable | Default | Description |
|----------|---------|-------------|
| `SERVER_HOST` | `0.0.0.0` | Bind address |
| `SERVER_PORT` | `9000` | TCP port |
| `VERCEL_WEBHOOK_URL` | — | POST target for pose updates |
| `WEBHOOK_SECRET` | — | Bearer token for Vercel |
| `STREAM_IDLE_TIMEOUT_S` | `30` | Auto-end session if no sensor data |
| `STREAM_LATENCY_S` | `auto` | `auto` = adaptive; or fixed seconds (e.g. `0.5`) |
| `STREAM_MIN_LATENCY_S` | `0.05` | Adaptive latency floor |
| `STREAM_MAX_LATENCY_S` | `2.0` | Adaptive latency ceiling |
| `STREAM_OUTPUT_HZ` | `100` | Fused tick rate for async sensor stream |
| `FUSION_LIB_PATH` | `native/libfusion.so` | Override library path |
| `FUSION_CALIB_PATH` | `fusion_calib.json` | Saved flow lever-arm calibration |

## Lever-arm calibration (IMU + PMW3901)

The EKF compensates optical flow using rotational lever arms at **two** points:

| Offset | Meaning | Used in |
|--------|---------|---------|
| `imu_lever_arm_m` | Rotation center → IMU (gyro) | `omega x r` at IMU |
| `flow_lever_arm_m` | IMU → PMW3901 flow sensor | `omega x r` at flow |

When the collar spins about its physical center (not the IMU chip), both offsets matter:

`v_flow = v_state + omega x imu_lever_arm + omega x flow_lever_arm`

Both values are saved to `fusion_calib.json` and loaded when the server starts.

### Before you begin

1. Rebuild the native library after pulling: `./build_lib.sh`
2. Use a **textured, non-reflective** surface under the flow sensor
3. Place the collar **flat** on the surface
4. Identify the device **rotation center** (usually the physical center of the collar)
5. You will rotate about the body **+X axis** (right) at roughly **0.04 rad/s** (~2.3°/s)

### What you need per sensor message

During calibration, each bundled `sensor` line must include:

```json
{
  "type": "sensor",
  "ts_us": 1234567890123,
  "gyro": {"x": 0.04, "y": 0.0, "z": 0.0},
  "accel": {"x": 0.0, "y": 0.05, "z": -0.02},
  "flow": {"dx": 0, "dy": -3, "quality": 255},
  "range": {"mm": 550}
}
```

- **gyro** — confirms steady rotation rate
- **accel** — estimates IMU offset from rotation center (centripetal)
- **flow** — estimates flow-sensor offset from IMU
- **range** — height for flow velocity scaling

You do **not** need `start`/`end` during calibration-only mode.

### Calibrate from the collar (no PC scripts)

If the collar (or a minimal serial→TCP bridge) connects directly to the server,
send **control lines** inline with sensor data. The server runs calibration;
no `collar_calibrate.py` on the PC.

**Calibration sequence:**

```
TCP connect → oracle_ip:9000

CAL_START                          ← plain text line, or [99,0,[1]]

[0, 1234, [x, y, z, w]]            ← sensor stream while spinning ~5+ s
[2, 1235, [dx, dy, 255]]
[3, 1236, [550]]
...

CAL_FINISH                         ← plain text line, or [99,0,[2]]
```

Disconnecting mid-calibration also triggers auto-finish on the server.

**Live tracking after calibration:**

```
STREAM_START                       ← or [99,0,[10]]
... sensor lines ...
STREAM_END                         ← or [99,0,[11]]
```

| Control | Plain text | Wire array |
|---------|------------|------------|
| Begin calibration | `CAL_START` | `[99, 0, [1]]` |
| Finish calibration | `CAL_FINISH` | `[99, 0, [2]]` |
| Cancel calibration | `CAL_CANCEL` | `[99, 0, [3]]` |
| Begin fusion stream | `STREAM_START` | `[99, 0, [10]]` |
| End fusion stream | `STREAM_END` | `[99, 0, [11]]` |

`$` prefix works too (`$CAL_START`). Axis detection and saving `fusion_calib.json`
are handled entirely on the server.

### Optional: serial bridge on PC

If the collar only has USB serial (no TCP), a minimal bridge can forward lines
unchanged — control lines pass through as-is:

```bash
pip install pyserial
python3 collar_stream.py --serial /dev/ttyACM0 --host <ORACLE_IP>
```

The bridge does **not** run calibration logic; firmware should emit
`CAL_START` / sensor data / `CAL_FINISH` on the serial port.

### Optional: dev/test scripts

`collar_calibrate.py` and `cal_lever_arm.py` remain for development and file
replay; they are not required for production collar firmware.

### Live collar calibration (legacy script)

Use when the collar is connected to your laptop via USB and the fusion server runs on Oracle.

**Prerequisites**

1. Server built and running (`./build_lib.sh`, port 9000 open).
2. Collar firmware emits JSONL lines (one sensor per line) on USB serial.
3. Textured surface; collar flat with flow sensor pointing down.

**Steps**

```bash
# On laptop (install once)
pip install pyserial

# Find port: Device Manager → COM3 (Windows) or ls /dev/ttyACM* (Linux)

# Run calibration — spin the collar about ONE axis for ~5+ seconds when prompted
py collar_calibrate.py --serial COM3 --host <ORACLE_PUBLIC_IP> --duration 30

# Linux
python3 collar_calibrate.py --serial /dev/ttyACM0 --host <ORACLE_IP> --duration 30
```

The server **auto-detects** which body axis (x/y/z) you rotated about from gyro data.
Results are written to `fusion_calib.json` on the server.

**After calibration — live position streaming to the website**

```bash
py collar_stream.py --serial COM3 --host <ORACLE_IP>
```

Move the collar on the table; fused poses POST to Vercel automatically.

**Collar JSONL format** (what firmware should print):

```json
{"kind":"quat","t_ms":12345,"quat":{"w":1,"x":0,"y":0,"z":0}}
{"kind":"accel","t_ms":12345,"accel_mps2":{"x":0,"y":0,"z":9.8}}
{"kind":"flow","t_ms":12350,"flow":{"delta_x":1,"delta_y":0,"quality":255}}
{"kind":"range","t_ms":12360,"filtered":{"distance_mm":550,"valid":true}}
```

**Troubleshooting**

| Symptom | Fix |
|---------|-----|
| `not enough valid samples` | Spin longer and steadier about one axis only |
| `axis_locked: false` in status | Keep rotating until status shows detected axis |
| No serial port | Install driver; check USB cable (data, not charge-only) |
| Server connection refused | Open Oracle port 9000; confirm server is running |

### Option A: Automated script (file replay)

```bash
./build_lib.sh
python3 fusion_server.py &

# Rotate the collar about +X on the table while this runs (~3+ seconds of data)
python3 cal_lever_arm.py capture.jsonl --host 127.0.0.1 --axis x --omega 0.04
```

The script resamples your capture to 100 Hz, starts calibration, streams data, and finishes.

### Option B: Manual TCP sequence

```bash
# 1. Start calibration
{"type":"cal_lever_arm_start","axis":"x","omega_rad_s":0.04}

# 2. Stream sensor lines at ~100 Hz for 3+ seconds while rotating steadily
{"type":"sensor", ...}

# 3. Finish and save
{"type":"cal_lever_arm_finish"}

# 4. Check result
{"type":"cal_lever_arm_status"}
```

### Physical procedure

1. Put the collar flat on a textured surface, flow sensor pointing down
2. Start `cal_lever_arm_start` (axis `x`, omega `0.04`)
3. Spin the collar smoothly about its **right (+X) axis** through the rotation center
   - Keep rotation rate steady (~0.04 rad/s)
   - Avoid translating the collar across the table
   - Keep the collar flat (don't lift one edge)
4. Stream sensor data for at least **3 seconds** (300+ samples at 100 Hz)
5. Send `cal_lever_arm_finish`
6. Restart the server (or it picks up values immediately after finish)

### Expected output (`fusion_calib.json`)

```json
{
  "imu_lever_arm_m": {"x": 0.0, "y": 0.012, "z": -0.008},
  "flow_lever_arm_m": {"x": 0.0, "y": 0.0, "z": 0.045},
  "calibrated_at_ms": 1730000000000,
  "axis": "x",
  "omega_rad_s": 0.04,
  "samples_used": 312,
  "residual_rms_mps": 0.002
}
```

### Troubleshooting

| Symptom | Fix |
|---------|-----|
| `not enough valid samples` | Rotate longer, keep rate steadier, check all 4 sensor fields present |
| High `samples_rejected` | Rotation too fast/slow vs `omega_rad_s`; reduce wobble on other axes |
| `residual_rms_mps` large | Surface too smooth, range invalid, or collar not flat |
| Values look wrong | Confirm rotation is about **+X** and center of rotation matches device center |

### Optional: calibrate other axes

| Axis | Estimates |
|------|-----------|
| `x` (right, default) | `imu_lever_arm` y/z + `flow_lever_arm` z |
| `y` | `imu_lever_arm` x/z + `flow_lever_arm` z |
| `z` | `imu_lever_arm` x/y + `flow_lever_arm` x/y |

Run separate calibrations per axis if you need the full 3D offset for each sensor.

### Persist on Oracle

```bash
# In /etc/fusion-server/env
FUSION_CALIB_PATH=/home/ubuntu/fusion_server/fusion_calib.json
```

Copy `fusion_calib.json` to that path after calibration. The server loads it on boot.
