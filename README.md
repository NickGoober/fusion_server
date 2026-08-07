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

Sensor fields (all optional per line, but send a full set each tick for best fusion):

```json
{
  "type": "sensor",
  "ts_us": 1234567890123,
  "quat": {"w": 1, "x": 0, "y": 0, "z": 0},
  "gyro": {"x": 0, "y": 0, "z": 0},
  "accel": {"x": 0, "y": 0, "z": 0},
  "flow": {"dx": 0, "dy": -5, "quality": 255},
  "range": {"mm": 550, "strength": 100}
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
