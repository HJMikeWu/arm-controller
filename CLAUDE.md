# Project: Turin Single-Arm Controller

## Hardware

| Device | IP | Notes |
|--------|-----|-------|
| Arm | 192.168.1.101 | Turin TCR-010, TCP port 8527 |

## Running

```bash
# Linux (MIC-733)
.venv/bin/python3 -u arm_ui.py

# macOS
DYLD_LIBRARY_PATH=/opt/homebrew/Cellar/expat/2.8.0/lib .venv/bin/python3 -u arm_ui.py
```

## Turin Robot Known Limitations

- **Only `MoveL` works** (Cartesian linear). `MoveJ`, `MoveAbsJ`, PTP all fail with MotionControlMode=48.
- TCP jog: motion=3 (tool frame), axes ±1/2/3 = X/Y/Z.
- Joint jog: motion=1, axes ±1–6 = J1–J6. Requires TarPos; use ±360 as soft limit.
- `jog_stop` must use `abs(axis)` — negative axis not accepted by Stop command.
- `is_idle()` comparison must be case-insensitive (robot returns "Stopped" or "stopped").

## DO9

- DO9 is the hand/auto switch: DO9=1 → auto mode, DO9=0 → manual mode.
- Toggled by the AUTO/MANUAL button in the UI.

## Home Joints

`[-23.038, 86.39, -40.894, 0.042, 45.525, 37.733]` (stored in `config/settings.yaml`)
