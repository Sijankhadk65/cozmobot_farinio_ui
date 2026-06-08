"""Robot connection + motion, wrapping the bundled Fairino SDK.

Mirrors the robot logic from test_weld_line_measure.py: connect/enable/reset,
home (MoveJ to START_JOINTS), and move-along-line (resolve length+angle into
dx/dy and run a linear or weave MoveL). All methods are blocking and meant to be
called from a worker thread so the GUI stays responsive.
"""

import sys
import math
import time
from pathlib import Path

# The fairino SDK package lives next to this file. Point at it and import Robot,
# matching the bundled-SDK import pattern used across the project.
_SDK_DIR = Path(__file__).resolve().parent / "fairino"
if str(_SDK_DIR) not in sys.path:
    sys.path.insert(0, str(_SDK_DIR))
import Robot  # noqa: E402

# Start/home configuration in joint angles [j1..j6] in degrees.
START_JOINTS = [-90.0, -90.0, 85.0, -85.0, -90.0, 0.0]


def _fmt_pose(p):
    """Compact [x, y, z, rx, ry, rz] formatter for debug logs."""
    return "[" + ", ".join(f"{v:.1f}" for v in p) + "]"


def _default_log(msg, level="info"):
    """Fallback logger used when no UI logger is injected."""
    print(f"[{level}] {msg}")


def _ret_level(ret):
    """Map an SDK return code to a log level (0 = success, else error)."""
    return "success" if ret == 0 else "error"


class RobotController:
    """Holds the live RPC connection and exposes the GUI's robot actions.

    The injected ``log`` callable takes ``(msg, level="info")``; levels drive the
    UI's color-coded log (info/success/warn/error/dryrun/header).
    """

    def __init__(self, log=_default_log):
        self.robot = None
        self.tool = 0
        self.user = 0
        self.log = log
        # Move tuning (matches the reference defaults).
        self.angle_offset_deg = 90.0
        # Global velocity (percentage of max speed) applied to every move —
        # home, linear, and weave. Set from the UI velocity input.
        self.velocity = 20.0
        # When True, moves log their planned start/target/params but send no
        # motion command to the robot. Toggled from the UI for safe debugging.
        self.dry_run = False

    @property
    def connected(self):
        return self.robot is not None

    # --- connection ---

    def connect(self, ip):
        """Connect, switch to auto mode, enable, clear faults, read active frames."""
        self.log(f"Connecting to {ip} ...")
        robot = Robot.RPC(ip)
        time.sleep(0.5)
        robot.Mode(0)
        time.sleep(0.5)
        robot.RobotEnable(1)
        time.sleep(1.0)
        robot.ResetAllError()
        time.sleep(0.5)

        self.tool = robot.GetActualTCPNum()[1]
        self.user = robot.GetActualWObjNum()[1]
        self.robot = robot
        self.log(f"Connected. Active tool={self.tool}, workpiece={self.user}.", "success")
        return self.tool, self.user

    def disconnect(self):
        if self.robot is not None:
            try:
                self.robot.CloseRPC()
            except Exception:
                pass
        self.robot = None
        self.log("Disconnected.")

    def get_state(self):
        """Non-blocking snapshot of TCP pose, joints, and fault codes.

        Reads the SDK's realtime state struct (kept current by its background
        socket thread), so it's cheap enough to poll from the UI. Returns a dict
        {"tcp", "joints", "fault"} or None if not connected.
        """
        if self.robot is None:
            return None
        return {
            "tcp": self.robot.GetActualTCPPose()[1],          # [x,y,z,rx,ry,rz]
            "joints": self.robot.GetActualJointPosDegree()[1],  # [j1..j6] deg
            "fault": self.robot.GetRobotErrorCode()[1],         # [main, sub]
        }

    # --- motions ---

    def home(self):
        """Joint move to the start configuration (no IK needed)."""
        self.log(f"Homing -> {START_JOINTS}", "header")
        if self.dry_run:
            self.log(f"would MoveJ to {START_JOINTS} @ vel={self.velocity}%", "dryrun")
            return None
        ret = self.robot.MoveJ(START_JOINTS, self.tool, self.user, vel=self.velocity)
        self.log(f"MoveJ home returned {ret}", _ret_level(ret))
        return ret

    def linear_move(self, dx=0.0, dy=0.0, dz=0.0):
        start_pose = self.robot.GetActualTCPPose()[1]
        target = list(start_pose)
        target[0] += dx
        target[1] += dy
        target[2] += dz
        if self.dry_run:
            self.log(f"start={_fmt_pose(start_pose)} "
                     f"delta(dx={dx:.1f}, dy={dy:.1f}, dz={dz:.1f})", "dryrun")
            self.log(f"would MoveL -> {_fmt_pose(target)} @ vel={self.velocity}%", "dryrun")
            return None
        ret = self.robot.MoveL(desc_pos=target, tool=self.tool, user=self.user,
                               vel=self.velocity)
        self.log(f"MoveL -> {_fmt_pose(target)} returned {ret}", _ret_level(ret))
        return ret

    def weave_move(self, dx, dy, dz, n_weaves=10, speed_mm_s=20.0,
                   weave_range=10.0, weave_type=0, weave_num=0):
        """Linear move with `n_weaves` weave cycles (dry-run weld swing, no arc).

        Weave count is set by frequency, not directly:
            weaves = freq(Hz) * travel_time(s) = freq * path_len / speed
        so freq = n_weaves * speed / path_len.
        """
        weave_num = int(weave_num)
        path_len = math.sqrt(dx * dx + dy * dy + dz * dz)
        if path_len == 0:
            self.log("weave_move: zero-length path, nothing to do", "warn")
            return None
        weave_freq = n_weaves * speed_mm_s / path_len
        self.log(f"path_len={path_len:.1f}mm speed={speed_mm_s}mm/s "
                 f"-> {n_weaves} weaves @ {weave_freq:.3f} Hz")

        start_pose = self.robot.GetActualTCPPose()[1]
        target = list(start_pose)
        target[0] += dx
        target[1] += dy

        if self.dry_run:
            self.log(f"WeaveSetPara(num={weave_num}, type={weave_type}, "
                     f"freq={weave_freq:.3f}Hz, range={weave_range}mm)", "dryrun")
            self.log(f"start={_fmt_pose(start_pose)}", "dryrun")
            self.log(f"would weave MoveL -> {_fmt_pose(target)} @ vel={self.velocity}%", "dryrun")
            return None

        rc = self.robot.WeaveSetPara(weave_num, weave_type, weave_freq, 0,
                                     weave_range, 0, 0, 0, 0, 0, 0, 0)
        self.log(f"WeaveSetPara returned {rc}", _ret_level(rc))
        self.log(f"WeaveStart returned {self.robot.WeaveStart(weave_num)}")
        ret = self.robot.MoveL(desc_pos=target, tool=self.tool, user=self.user,
                               vel=self.velocity)
        self.log(f"MoveL (weaving, no arc) returned {ret}", _ret_level(ret))
        self.log(f"WeaveEnd returned {self.robot.WeaveEnd(weave_num)}")
        return ret

    def move_along_line(self, m, weave=False, weave_type=0):
        """Move along a measured line, splitting length into X and Y.

        dx = length*cos(angle), dy = length*sin(angle); the measured angle is
        rotated by angle_offset_deg to align the camera direction with the robot.
        `weave=True` runs the linear path with a weave swing (otherwise plain
        MoveL); `weave_type` picks the swing pattern (0=triangular, 4=sinusoidal).
        """
        if "length_mm" not in m:
            self.log("No metric measurement (set plane depth). Move skipped.", "warn")
            return None

        length = m["length_mm"]
        angle_deg = m.get("angle_cam", m["angle_visual"]) + self.angle_offset_deg
        angle_rad = math.radians(angle_deg)
        dx = length * math.cos(angle_rad)
        dy = length * math.sin(angle_rad)

        self.log(f"Move along line: length={length:.1f}mm "
                 f"angle={angle_deg:.1f}deg (+{self.angle_offset_deg:.0f} offset) "
                 f"-> dx={dx:.1f}, dy={dy:.1f}", "header")
        if weave:
            ret = self.weave_move(dx, dy, 0.0, weave_type=weave_type)
        else:
            ret = self.linear_move(dx, dy, 0.0)
        self.log("Move complete.", "success")
        return ret

    def move_along_lines(self, measurements, weave=False, weave_type=0):
        """Move along several measured lines back-to-back.

        Each line is a relative delta from the current TCP pose (same model as
        move_along_line), so the moves chain: line N starts where line N-1 ended.
        """
        rets = []
        n = len(measurements)
        for i, m in enumerate(measurements, 1):
            self.log(f"Line {i}/{n}", "header")
            rets.append(self.move_along_line(m, weave, weave_type))
        self.log(f"All {n} line move(s) complete.", "success")
        return rets
