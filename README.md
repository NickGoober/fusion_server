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

## Protocol (newline-delimited JSON)

| Message | Purpose |
|---------|---------|
| `{"type":"start"}` | Reset filter, begin streaming session |
| `{"type":"sensor", ...}` | Submit IMU / flow / range sample |
| `{"type":"end"}` | End session; webhook gets `streaming: false` |
| `{"type":"cal_lever_arm_start","axis":"x","omega_rad_s":0.04}` | Begin optical-flow lever-arm calibration |
| `{"type":"cal_lever_arm_finish"}` | Compute, apply, and save lever arm |
| `{"type":"cal_lever_arm_cancel"}` | Abort calibration |
| `{"type":"cal_lever_arm_status"}` | Query calibration progress / current arm |

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
| `STREAM_IDLE_TIMEOUT_S` | `3` | Auto-end session if no sensor data |
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

### Option A: Automated script (recommended)

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
