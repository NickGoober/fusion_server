# Fusion TCP server (Oracle Ubuntu)

Runs the Raedir EKF fusion stack via `libfusion.so` and forwards fused poses to Vercel.

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

End-to-end path:

```
Collar (USB) → laptop running collar_stream.py → Oracle fusion_server → Vercel webhook → website
```

### 1. Server (Oracle)

```bash
cd ~/fusion_server
git pull
./build_lib.sh

# /etc/fusion-server/env (or export manually)
SERVER_HOST=0.0.0.0
SERVER_PORT=9000
VERCEL_WEBHOOK_URL=https://your-app.vercel.app/api/gadget
WEBHOOK_SECRET=your-shared-secret
STREAM_LATENCY_S=auto          # adaptive (default); or e.g. 0.5 for fixed 500 ms
STREAM_IDLE_TIMEOUT_S=30       # keep session alive between sensor bursts
STREAM_OUTPUT_HZ=100

python3 fusion_server.py
# or: sudo systemctl restart fusion-server
```

Open TCP **9000** in Oracle Cloud security list and `ufw`.

### 2. Bridge laptop (collar connected via USB)

```bash
pip install pyserial   # once, for USB serial mode

# Linux — find port: ls /dev/ttyACM* /dev/ttyUSB*
python3 collar_stream.py --serial /dev/ttyACM0 --host <ORACLE_PUBLIC_IP>

# Windows
py collar_stream.py --serial COM3 --host <ORACLE_PUBLIC_IP>

# If firmware prints JSONL to stdout instead of a raw serial API:
./your_collar_app --jsonl | python3 collar_stream.py --stdin --host <ORACLE_PUBLIC_IP>
```

`collar_stream.py` sends `{"type":"start"}`, converts collar JSONL to `[sensor_type, timestamp, data_array]`, and sends `{"type":"end"}` on Ctrl+C.

If the collar already emits the stream format directly, lines pass through unchanged:

```json
[0, 1234, [0.1, 0.2, 0.3, 0.99]]
[2, 1235, [1, -2, 255]]
[3, 1236, [550]]
```

### 3. Collar JSONL format (one sensor per line)

```json
{"kind":"quat","t_ms":12345,"quat":{"w":1,"x":0,"y":0,"z":0}}
{"kind":"accel","t_ms":12345,"accel_mps2":{"x":0,"y":0,"z":9.8}}
{"kind":"flow","t_ms":12350,"flow":{"delta_x":1,"delta_y":0,"quality":255}}
{"kind":"range","t_ms":12360,"filtered":{"distance_mm":550,"valid":true}}
```

Sensor indices on the wire:

| Type | Sensor | Data array |
|------|--------|------------|
| 0 | Accel m/s² `[x, y, z]` | or quat `[x, y, z, w]` (4 values → quaternion) |
| 1 | Quaternion | `[x, y, z, w]` |
| 2 | Optical flow | `[dx, dy, quality]` |
| 3 | Radar range | `[mm]` |

### 4. Adaptive latency

The server measures each sensor's update interval and sets buffer latency to ~1.5× the slowest sensor period (min 50 ms, max 2 s). This replaces the old fixed 4 s delay.

Query current latency:

```json
{"type":"stream_status"}
```

Force a fixed delay instead:

```bash
export STREAM_LATENCY_S=0.5   # 500 ms fixed
```

### 5. Verify Vercel updates

- Watch server logs for `Webhook OK` or errors
- Move the collar on a textured surface; the website cube should track position
- Synthetic test without hardware: `python3 client_example.py` (bundled format, no latency buffer)

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

### Live collar calibration (auto axis)

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
