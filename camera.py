"""Open the Astra Pro color stream (a standard UVC webcam) via OpenCV.

The Astra's COLOR stream is plain UVC, so cv.VideoCapture works. Depth needs a
separate SDK and is not used here. Auto-detect finds the device by sysfs name so
it survives replug/reboot.
"""

import os
import glob

import cv2 as cv
import numpy as np

CAMERA_NAME = "Astra Pro"   # substring matched against /dev/video* device names
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


def apply_color_temp(frame, temp):
    """Shift a BGR frame's color temperature to cancel a warm/cool tint.

    `temp` runs roughly [-100, 100]: negative cools the image (the Astra Pro's
    warm cast is corrected with negative values), positive warms it, 0 is a
    no-op. Implemented as complementary red/blue channel gains.
    """
    if not temp:
        return frame
    factor = temp / 100.0 * 0.5   # ±50% max gain at the extremes
    out = frame.astype(np.float32)
    out[:, :, 2] *= 1.0 + factor  # red  channel (BGR -> index 2)
    out[:, :, 0] *= 1.0 - factor  # blue channel (BGR -> index 0)
    return np.clip(out, 0, 255).astype(np.uint8)


def find_camera_indices(name=CAMERA_NAME):
    """Return /dev/videoN indices whose sysfs name matches `name` (Linux)."""
    candidates = []
    for path in sorted(glob.glob("/sys/class/video4linux/video*/name")):
        try:
            with open(path) as f:
                dev_name = f.read().strip()
        except OSError:
            continue
        if name.lower() in dev_name.lower():
            idx = int(os.path.basename(os.path.dirname(path)).removeprefix("video"))
            candidates.append(idx)
    return candidates


def open_camera(index="auto", name=CAMERA_NAME,
                width=FRAME_WIDTH, height=FRAME_HEIGHT, log=print):
    """Open the configured camera and return a usable VideoCapture, or None.

    `index="auto"` finds the camera by name; an explicit int forces /dev/videoN.
    """
    if isinstance(index, int):
        indices = [index]
    else:
        indices = find_camera_indices(name)
        if not indices:
            log(f"No camera matching '{name}' found. "
                f"Set an explicit /dev/videoN index.")
            return None
        log(f"Auto-detected '{name}' at /dev/video{indices}.")

    for idx in indices:
        cap = cv.VideoCapture(idx, cv.CAP_V4L2)
        if not cap.isOpened():
            cap = cv.VideoCapture(idx)  # fall back to default backend
        if not cap.isOpened():
            continue
        cap.set(cv.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, height)
        ok, frame = cap.read()  # verify this node delivers actual color frames
        if ok and frame is not None and frame.ndim == 3:
            log(f"Using /dev/video{idx}.")
            return cap
        cap.release()

    log(f"Found {name} but no node delivered a color frame (tried {indices}).")
    return None
