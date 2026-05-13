# arm-controller

Turin TCR-010 single-arm controller UI built with OpenCV.

## Features

- **Jog XYZ** — TCP frame (+X/-X, +Y/-Y, +Z/-Z), hold button to jog, release to stop
- **Jog Joints** — J1–J6 joint frame
- **HOME** — move to home joints defined in config
- **SAVE POS / GO POS** — record current TCP position and move back to it
- **DO9 toggle** — AUTO / MANUAL mode switch via digital output
- **Alarm indicator + CLEAR ERR**
- **QUIT** button to exit cleanly
- Background TCP connect with auto-retry (UI opens immediately)

## Hardware

| Device | IP | Notes |
|--------|-----|-------|
| Turin arm | 192.168.0.103 | TCP port 8527 |

## Setup

**Linux (MIC-733):**
```bash
bash setup.sh
```

**macOS:**
```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Run

**Linux:**
```bash
.venv/bin/python3 -u arm_ui.py
```

**macOS:**
```bash
DYLD_LIBRARY_PATH=/opt/homebrew/Cellar/expat/2.8.0/lib .venv/bin/python3 -u arm_ui.py
```

## Config

Edit `config/settings.yaml` to set arm IP, port, and home joints:

```yaml
arm:
  ip: "192.168.0.103"
  port: 8527
  home_joints: [-23.038, 86.39, -40.894, 0.042, 45.525, 37.733]
  saved_pos: null  # written automatically by SAVE POS button
```

## Turin Robot Notes

- Only `MoveL` works (Cartesian linear). `MoveJ` fails with MotionControlMode=48.
- `jog_stop` requires `abs(axis)` — Turin rejects negative axis for Stop.
- Status comparison is case-insensitive (robot returns `"Stopped"` or `"stopped"`).
- DO9=1 → AUTO mode, DO9=0 → MANUAL mode.
