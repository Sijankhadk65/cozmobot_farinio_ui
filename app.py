"""PySide6 GUI for the Fairino line-follow demo.

Features:
  1. IP input + Connect button (connect / enable / reset the robot).
  2. Live camera view (Astra Pro color stream).
  3. Detect Line button (measure the line in the current frame).
  4. Go Home button (MoveJ to the start configuration).
  5. Move Along Line button (move the robot the line's length & direction).

The camera runs on a QTimer in the UI thread; every robot command runs on a
worker QThread so a multi-second MoveL never freezes the video or the window.
"""

import html
import sys
import time

import cv2 as cv
import numpy as np
from PySide6.QtCore import Qt, QThread, QTimer, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFrame, QGroupBox, QHBoxLayout,
    QLabel, QLineEdit, QListWidget, QListWidgetItem, QMainWindow, QPushButton,
    QSlider, QTextEdit, QVBoxLayout, QWidget,
)

import vision
from camera import apply_color_temp, open_camera, FRAME_WIDTH, FRAME_HEIGHT
from robot_control import RobotController


class Task(QThread):
    """Run one blocking callable off the UI thread; emit done/failed."""

    done = Signal(object)
    failed = Signal(str)

    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self._fn, self._args, self._kwargs = fn, args, kwargs

    def run(self):
        try:
            self.done.emit(self._fn(*self._args, **self._kwargs))
        except Exception as exc:  # surface SDK/runtime errors to the log
            import traceback
            traceback.print_exc()
            self.failed.emit(str(exc))


class MainWindow(QMainWindow):
    # Emitted for every log line. Logging is routed through a signal so worker
    # threads can log safely: Qt widgets may only be touched from the GUI thread,
    # and a signal emitted from another thread is delivered as a queued call on
    # the GUI thread. Calling appendPlainText directly from a worker thread is a
    # data race on the widget's document and crashes the app.
    log_message = Signal(str, str)

    # Per-level styling for the log view: (text color, extra CSS, tag label).
    # Tags are fixed-width so the message column stays aligned.
    LOG_STYLES = {
        "info":    ("#bdc3c7", "",                     "INFO"),
        "success": ("#2ecc71", "font-weight:bold;",    " OK "),
        "warn":    ("#f39c12", "",                     "WARN"),
        "error":   ("#e74c3c", "font-weight:bold;",    "ERR "),
        "dryrun":  ("#9b59b6", "font-style:italic;",   "DRY "),
        "header":  ("#3498db", "font-weight:bold;",    "»»»»"),
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Fairino Line Follow")
        self.log_message.connect(self._append_log)

        self.robot = RobotController(log=self.log)
        self.cap = None
        self.current_frame = None     # latest BGR frame (after tint correction)
        self.detections = []          # list of {"line": ..., "m": ...} per line
        self.show_mask = False
        self.color_temp = 0           # tint correction; <0 cools the warm cast
        self.task = None              # keep a ref so the QThread isn't GC'd

        self._build_ui()
        self._open_camera()
        self._refresh_buttons()

        # Camera refresh loop (~33 fps).
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update_frame)
        self.timer.start(30)

        # Robot state poll (~2 Hz) for the live pose panel.
        self.state_timer = QTimer(self)
        self.state_timer.timeout.connect(self._update_state)
        self.state_timer.start(500)

    # --- UI construction ---

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # Top: connection row.
        conn = QHBoxLayout()
        conn.addWidget(QLabel("Robot IP:"))
        self.ip_edit = QLineEdit("192.168.58.2")
        self.ip_edit.setFixedWidth(160)
        conn.addWidget(self.ip_edit)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect)
        conn.addWidget(self.connect_btn)
        self.status_lbl = QLabel("● Disconnected")
        self.status_lbl.setStyleSheet("color: #c0392b; font-weight: bold;")
        conn.addWidget(self.status_lbl)
        conn.addStretch()
        root.addLayout(conn)

        # Middle: camera (left) + controls (right).
        mid = QHBoxLayout()

        self.video_lbl = QLabel()
        self.video_lbl.setFixedSize(FRAME_WIDTH, FRAME_HEIGHT)
        self.video_lbl.setAlignment(Qt.AlignCenter)
        self.video_lbl.setStyleSheet("background:#111; color:#888;")
        self.video_lbl.setText("camera starting...")
        mid.addWidget(self.video_lbl)

        controls = QVBoxLayout()

        self.detect_btn = QPushButton("Detect Line")
        self.detect_btn.clicked.connect(self._on_detect)
        self.clear_btn = QPushButton("Clear Detections")
        self.clear_btn.clicked.connect(self._on_clear)
        self.home_btn = QPushButton("Go Home")
        self.home_btn.clicked.connect(self._on_home)
        self.move_btn = QPushButton("Move Along Line")
        self.move_btn.clicked.connect(self._on_move_linear)
        self.weave_btn = QPushButton("Move Along Line (Weave)")
        self.weave_btn.clicked.connect(self._on_move_weave)
        self.sine_btn = QPushButton("Move Along Line (Sine Weave)")
        self.sine_btn.clicked.connect(self._on_move_sine)
        for b in (self.detect_btn, self.clear_btn, self.home_btn, self.move_btn,
                  self.weave_btn, self.sine_btn):
            b.setMinimumHeight(44)
            controls.addWidget(b)

        # Options group.
        opts = QGroupBox("Options")
        opts_l = QVBoxLayout(opts)

        color_row = QHBoxLayout()
        color_row.addWidget(QLabel("Line color:"))
        self.color_box = QComboBox()
        self.color_box.addItems(list(vision.COLOR_RANGES.keys()))
        self.color_box.setCurrentText(vision.TARGET_COLOR)
        self.color_box.currentTextChanged.connect(self._on_color_changed)
        color_row.addWidget(self.color_box)
        color_row.addStretch()
        opts_l.addLayout(color_row)

        depth_row = QHBoxLayout()
        depth_row.addWidget(QLabel("Plane depth (mm):"))
        self.depth_spin = QDoubleSpinBox()
        self.depth_spin.setRange(0.0, 5000.0)
        self.depth_spin.setDecimals(1)
        self.depth_spin.setValue(vision.PLANE_DEPTH_MM)
        self.depth_spin.setKeyboardTracking(False)  # don't emit mid-typing
        # Apply only when Enter is pressed, not on every keystroke or step.
        self.depth_spin.lineEdit().returnPressed.connect(self._on_depth_changed)
        depth_row.addWidget(self.depth_spin)
        depth_row.addStretch()
        opts_l.addLayout(depth_row)

        vel_row = QHBoxLayout()
        vel_row.addWidget(QLabel("Velocity (%):"))
        self.vel_spin = QDoubleSpinBox()
        self.vel_spin.setRange(1.0, 100.0)
        self.vel_spin.setDecimals(1)
        self.vel_spin.setValue(self.robot.velocity)
        self.vel_spin.setKeyboardTracking(False)  # don't emit mid-typing
        # Apply only when Enter is pressed, not on every keystroke or step.
        self.vel_spin.lineEdit().returnPressed.connect(self._on_velocity_changed)
        vel_row.addWidget(self.vel_spin)
        vel_row.addStretch()
        opts_l.addLayout(vel_row)

        # Color-temperature slider to cancel the Astra Pro's warm tint.
        temp_row = QHBoxLayout()
        temp_row.addWidget(QLabel("Tint:"))
        self.temp_slider = QSlider(Qt.Horizontal)
        self.temp_slider.setRange(-100, 100)
        self.temp_slider.setValue(self.color_temp)
        self.temp_slider.setToolTip("Cooler (cancel warm tint) ↔ warmer")
        self.temp_slider.valueChanged.connect(self._on_temp_changed)
        # Log only the settled value on release, not every intermediate step.
        self.temp_slider.sliderReleased.connect(self._on_temp_released)
        temp_row.addWidget(self.temp_slider, 1)
        self.temp_lbl = QLabel("0")
        self.temp_lbl.setFixedWidth(32)
        temp_row.addWidget(self.temp_lbl)
        opts_l.addLayout(temp_row)

        self.mask_btn = QPushButton("Show Mask: off")
        self.mask_btn.setCheckable(True)
        self.mask_btn.toggled.connect(self._on_mask_toggled)
        opts_l.addWidget(self.mask_btn)

        self.dryrun_btn = QPushButton("Dry Run: off")
        self.dryrun_btn.setCheckable(True)
        self.dryrun_btn.toggled.connect(self._on_dryrun_toggled)
        opts_l.addWidget(self.dryrun_btn)

        controls.addWidget(opts)

        # Measurement readout: one removable row per detection.
        controls.addWidget(QLabel("Detections:"))
        self.detect_list = QListWidget()
        self.detect_list.setFrameShape(QFrame.StyledPanel)
        self.detect_list.setMinimumHeight(110)
        controls.addWidget(self.detect_list)
        self._refresh_detection_list()

        # Live robot state (TCP pose / joints / fault), polled ~2 Hz.
        self.pose_lbl = QLabel("TCP: —\nJoints: —\nStatus: not connected")
        self.pose_lbl.setFrameShape(QFrame.StyledPanel)
        self.pose_lbl.setStyleSheet("padding:6px; font-family:monospace; font-size:11px;")
        controls.addWidget(self.pose_lbl)

        controls.addStretch()
        mid.addLayout(controls)
        root.addLayout(mid)

        # Bottom: log header row (label + clear) then a rich-text view so each
        # line can carry a timestamp, a color-coded severity tag, and headers.
        log_header = QHBoxLayout()
        log_header.addWidget(QLabel("Log:"))
        log_header.addStretch()
        self.clear_log_btn = QPushButton("Clear Logs")
        self.clear_log_btn.clicked.connect(self._on_clear_logs)
        log_header.addWidget(self.clear_log_btn)
        root.addLayout(log_header)

        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(500)
        self.log_view.setFixedHeight(140)
        self.log_view.setStyleSheet(
            "QTextEdit { background:#1e1e1e; font-family:monospace; font-size:11px; }")
        root.addWidget(self.log_view)

    # --- helpers ---

    def log(self, msg, level="info"):
        # Safe to call from any thread; the signal hops the message to the GUI
        # thread (queued when emitted from a worker, direct when on the GUI thread).
        self.log_message.emit(str(msg), level)

    def _append_log(self, msg, level="info"):
        color, extra, tag = self.LOG_STYLES.get(level, self.LOG_STYLES["info"])
        ts = time.strftime("%H:%M:%S")
        # Escape so message text containing >, <, & (e.g. ">>>", "->") renders
        # literally instead of being parsed as markup. NBSP keeps the tag aligned.
        tag = tag.replace(" ", "&nbsp;")
        body = html.escape(msg)
        self.log_view.append(
            f'<span style="color:#7f8c8d;">{ts}</span> '
            f'<span style="color:{color};font-weight:bold;">[{tag}]</span> '
            f'<span style="color:{color};{extra}">{body}</span>')

    def _on_clear_logs(self):
        self.log_view.clear()

    def _open_camera(self):
        self.cap = open_camera(log=self.log)
        if self.cap is None:
            self.video_lbl.setText("no camera")

    def _refresh_buttons(self):
        busy = self.task is not None and self.task.isRunning()
        connected = self.robot.connected
        have_metric = any("length_mm" in d["m"] for d in self.detections)

        self.connect_btn.setEnabled(not busy)
        self.detect_btn.setEnabled(not busy and self.current_frame is not None)
        self.clear_btn.setEnabled(not busy and bool(self.detections))
        self.home_btn.setEnabled(not busy and connected)
        self.move_btn.setEnabled(not busy and connected and have_metric)
        self.weave_btn.setEnabled(not busy and connected and have_metric)
        self.sine_btn.setEnabled(not busy and connected and have_metric)

    def _run(self, fn, *args):
        """Run a robot command on a worker thread, gating the UI while it runs."""
        self.task = Task(fn, *args)
        self.task.done.connect(self._on_task_done)
        self.task.failed.connect(self._on_task_failed)
        self.task.finished.connect(self._refresh_buttons)
        self._refresh_buttons()
        self.task.start()

    def _on_task_done(self, _result):
        self._refresh_buttons()

    def _on_task_failed(self, msg):
        self.log(msg, "error")
        self._refresh_buttons()

    # --- camera loop ---

    def _update_frame(self):
        if self.cap is None:
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return
        # Correct the warm tint before anything else so the view, the mask, and
        # detection all work off the same color-balanced frame.
        self.current_frame = apply_color_temp(frame, self.color_temp)
        frame = self.current_frame

        display = frame.copy()
        if self.show_mask:
            display[vision.build_color_mask(frame) > 0] = (0, 255, 255)
        if self.detections:
            for i, d in enumerate(self.detections, 1):
                vision.annotate(display, d["line"], d["m"], index=i)
        else:
            cv.putText(display, f"no {vision.TARGET_COLOR} line measured",
                       (12, FRAME_HEIGHT - 16), cv.FONT_HERSHEY_SIMPLEX, 0.6,
                       (0, 0, 255), 2)

        rgb = cv.cvtColor(display, cv.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        img = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        self.video_lbl.setPixmap(QPixmap.fromImage(img))

        if not self.detect_btn.isEnabled():
            self._refresh_buttons()  # enable Detect once first frame arrives

    def _update_state(self):
        if not self.robot.connected:
            self.pose_lbl.setText("TCP: —\nJoints: —\nStatus: not connected")
            return
        try:
            st = self.robot.get_state()
        except Exception:
            return  # realtime struct not populated yet; retry next tick
        if st is None:
            return
        tcp = "  ".join(f"{v:7.1f}" for v in st["tcp"])
        joints = "  ".join(f"{v:7.1f}" for v in st["joints"])
        main, sub = st["fault"]
        status = "OK" if main == 0 else f"FAULT main={main} sub={sub}"
        self.pose_lbl.setText(
            f"TCP  [x y z rx ry rz]: {tcp}\n"
            f"Jnts [j1..j6]:         {joints}\n"
            f"Status: {status}")

    # --- button handlers ---

    def _on_connect(self):
        if self.robot.connected:
            self.robot.disconnect()
            self._set_status(False)
            self.connect_btn.setText("Connect")
            self._refresh_buttons()
            return

        ip = self.ip_edit.text().strip()
        self.connect_btn.setText("Connect")

        def done(result):
            self._set_status(True)
            self.connect_btn.setText("Disconnect")
            self._on_task_done(result)
            # Auto-home once connected. Deferred to the next event-loop turn so the
            # connect QThread has fully finished before _run reuses self.task.
            self.log("Connected — auto-homing...", "success")
            QTimer.singleShot(0, self._on_home)

        self.task = Task(self.robot.connect, ip)
        self.task.done.connect(done)
        self.task.failed.connect(self._on_task_failed)
        self.task.finished.connect(self._refresh_buttons)
        self._refresh_buttons()
        self.task.start()

    def _on_detect(self):
        if self.current_frame is None:
            return
        lines, _, _ = vision.detect_lines(self.current_frame)
        if not lines:
            self.detections = []
            self._refresh_detection_list()
            self.log(f"No '{vision.TARGET_COLOR}' line detected. "
                     f"Toggle Show Mask to tune the color.", "warn")
            self._refresh_buttons()
            return

        self.detections = [{"line": ln, "m": vision.measure_line(ln)} for ln in lines]
        self.log(f"Detected {len(self.detections)} '{vision.TARGET_COLOR}' line(s)", "header")
        for i, d in enumerate(self.detections, 1):
            self.log(f"  [{i}] {self._row_text(d['m'])}")
        self._refresh_detection_list()
        self._refresh_buttons()

    def _on_clear(self):
        if not self.detections:
            return
        n = len(self.detections)
        self.detections = []
        self._refresh_detection_list()
        self.log(f"Cleared {n} detection(s).")
        self._refresh_buttons()

    @staticmethod
    def _row_text(m):
        """One-line human-readable summary of a measurement dict."""
        if "length_mm" in m:
            return f"{m['length_mm']:.1f} mm @ {m['angle_cam']:.1f}° ({m.get('type')})"
        return (f"{m['length_px']:.1f} px @ {m['angle_visual']:.1f}° "
                f"(set plane depth for mm)")

    def _refresh_detection_list(self):
        """Rebuild the detection list from self.detections.

        Rebuilt wholesale on every change so the [i] numbering stays in sync with
        the camera overlay after a removal, and remove buttons never hold a stale
        index (each captures its detection object, not its position).
        """
        self.detect_list.clear()
        if not self.detections:
            placeholder = QListWidgetItem("No line measured.")
            placeholder.setFlags(Qt.NoItemFlags)
            self.detect_list.addItem(placeholder)
            return
        for i, d in enumerate(self.detections, 1):
            row = QWidget()
            lay = QHBoxLayout(row)
            lay.setContentsMargins(4, 2, 4, 2)
            lbl = QLabel(f"[{i}] {self._row_text(d['m'])}")
            lbl.setWordWrap(True)
            lay.addWidget(lbl, 1)
            rm = QPushButton("✕")
            rm.setFixedWidth(28)
            rm.setToolTip("Remove this detection")
            rm.clicked.connect(lambda _checked=False, det=d: self._on_remove_detection(det))
            lay.addWidget(rm)

            item = QListWidgetItem()
            item.setSizeHint(row.sizeHint())
            self.detect_list.addItem(item)
            self.detect_list.setItemWidget(item, row)

    def _on_remove_detection(self, det):
        """Permanently drop one detection (matched by identity, not index)."""
        before = len(self.detections)
        self.detections = [d for d in self.detections if d is not det]
        if len(self.detections) != before:
            self.log(f"Removed detection: {self._row_text(det['m'])}")
        self._refresh_detection_list()
        self._refresh_buttons()

    def _on_home(self):
        self._run(self.robot.home)

    def _on_move_linear(self):
        self._move_detected(weave=False)

    def _on_move_weave(self):
        self._move_detected(weave=True, weave_type=0)   # planar triangular

    def _on_move_sine(self):
        self._move_detected(weave=True, weave_type=4)   # planar sinusoidal

    def _move_detected(self, weave, weave_type=0):
        metrics = [d["m"] for d in self.detections if "length_mm" in d["m"]]
        if not metrics:
            return
        self._run(self.robot.move_along_lines, metrics, weave, weave_type)

    # --- option handlers ---

    def _on_color_changed(self, color):
        vision.TARGET_COLOR = color
        self.log(f"Target color -> {color}")

    def _on_depth_changed(self):
        value = self.depth_spin.value()
        vision.PLANE_DEPTH_MM = value
        self.log(f"Plane depth -> {value:.1f} mm")
        # Depth scales mm length, so re-measure any existing detections from
        # their stored lines and refresh the list (otherwise the change only
        # takes effect on the next Detect Line press).
        if self.detections:
            for d in self.detections:
                d["m"] = vision.measure_line(d["line"])
            self._refresh_detection_list()

    def _on_velocity_changed(self):
        value = self.vel_spin.value()
        self.robot.velocity = value
        self.log(f"Velocity -> {value:.1f}%")

    def _on_temp_changed(self, value):
        self.color_temp = value
        self.temp_lbl.setText(str(value))

    def _on_temp_released(self):
        self.log(f"Tint -> {self.color_temp}")

    def _on_mask_toggled(self, checked):
        self.show_mask = checked
        self.mask_btn.setText(f"Show Mask: {'on' if checked else 'off'}")

    def _on_dryrun_toggled(self, checked):
        self.robot.dry_run = checked
        self.dryrun_btn.setText(f"Dry Run: {'on' if checked else 'off'}")
        if checked:
            self.log("Dry-run ENABLED — moves are logged, not executed.", "dryrun")
        else:
            self.log("Dry-run disabled — moves will execute.", "warn")

    def _set_status(self, connected):
        if connected:
            self.status_lbl.setText("● Connected")
            self.status_lbl.setStyleSheet("color:#27ae60; font-weight:bold;")
        else:
            self.status_lbl.setText("● Disconnected")
            self.status_lbl.setStyleSheet("color:#c0392b; font-weight:bold;")

    # --- shutdown ---

    def closeEvent(self, event):
        self.timer.stop()
        self.state_timer.stop()
        if self.cap is not None:
            self.cap.release()
        if self.robot.connected:
            self.robot.disconnect()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
