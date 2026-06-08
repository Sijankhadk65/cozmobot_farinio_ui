# Fairino Line-Follow GUI

PySide6 desktop app that watches the Astra Pro camera, detects a drawn line, and
moves the Fairino robot along that line's length and direction.

## Run

```bash
uv sync          # first time only
uv run main.py   # or: uv run app.py
```

## What the buttons do

- **Robot IP + Connect** — opens an RPC connection to the robot, switches to auto
  mode, enables it, and clears faults (`Mode(0)` → `RobotEnable(1)` →
  `ResetAllError`). Click again to disconnect. The camera runs independently and
  does not need the robot.
- **Detect Line** — runs the HSV-color → Sobel → contour → line-fit pipeline on
  the current frame, then measures the line's length and orientation. With a
  plane depth set it reports millimetres in the camera frame; otherwise pixels.
- **Go Home** — `MoveJ` to the start joint configuration
  (`START_JOINTS` in `robot_control.py`).
- **Move Along Line** — resolves the measured length+angle into dx/dy
  (`dx = L·cos θ`, `dy = L·sin θ`, with `angle_offset_deg` aligning camera→robot)
  and runs a weave or plain `MoveL`. Enabled only after a metric measurement.

## Options

- **Line color** — blue / red / green HSV ranges.
- **Plane depth (mm)** — distance from camera to the work surface; required for
  millimetre measurements (planar back-projection).
- **Show Mask** — tints matched pixels so you can tune the color live.

## Layout

| File | Role |
|------|------|
| `app.py` | PySide6 window, camera timer, worker threads |
| `vision.py` | line detection + measurement (pure, no robot/Qt) |
| `camera.py` | Astra Pro auto-detect + `VideoCapture` open |
| `robot_control.py` | RPC connect, home, move-along-line (bundled SDK) |
| `fairino/` | bundled Fairino Python SDK (`Robot.py`) |

The vision and robot logic is lifted from
`farino_app/test_weld_line_measure.py`. Each robot command runs on a worker
thread so a multi-second `MoveL` never freezes the video or the UI.

## Notes

- `CAMERA_MATRIX` in `vision.py` is a placeholder Astra default — calibrate with a
  checkerboard for accurate millimetres.
- The reference script descended 37 mm after homing to bring the tool near the
  surface; that step is intentionally left out of the GUI. Add it via the robot
  panel or a constant if your setup needs it.
