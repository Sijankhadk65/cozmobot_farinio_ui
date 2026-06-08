"""Line detection + measurement.

Lifted from test_weld_line_measure.py: HSV color mask -> morphology -> Sobel ->
contours -> line classification, then fit the line, find its endpoints, and
measure length/orientation in pixels and (with calibration) in the camera frame.

Self-contained: is_line_or_curve is copied in here so the GUI doesn't pull in
single_shape.py (which drags in matplotlib).
"""

import numpy as np
import cv2 as cv
from scipy.stats import linregress

# --- Tunable config (the GUI mutates TARGET_COLOR and PLANE_DEPTH_MM live) ---

# Target line colors in HSV. Red wraps the hue axis so it needs TWO ranges.
COLOR_RANGES = {
    "blue":  [((90, 50, 50), (130, 255, 255))],
    "red":   [((0, 60, 40), (12, 255, 255)), ((168, 60, 40), (180, 255, 255))],
    "green": [((40, 50, 50), (85, 255, 255))],
}
TARGET_COLOR = "red"

# Pinhole intrinsics [[fx,0,cx],[0,fy,cy],[0,0,1]] in pixels. Placeholder =
# classic PrimeSense/Astra color default at 640x480. CALIBRATE for accuracy.
CAMERA_MATRIX = np.array([[525.0, 0.0, 319.5],
                          [0.0, 525.0, 239.5],
                          [0.0, 0.0, 1.0]])
# Measured distance (mm) from camera to the plane the line sits on. None -> px only.
PLANE_DEPTH_MM = 443.50


def is_line_or_curve(contour, threshold=0.15):
    """Classify a contour as 'Line' or 'Curve' by deviation from a fitted line."""
    if len(contour) < 5:
        return "Line", 0.0

    x = contour[:, 0, 0].astype(float)
    y = contour[:, 0, 1].astype(float)
    slope, intercept, *_ = linregress(x, y)
    y_pred = slope * x + intercept
    residuals = np.abs(y - y_pred)

    bbox_diagonal = np.sqrt((x.max() - x.min()) ** 2 + (y.max() - y.min()) ** 2)
    if bbox_diagonal == 0:
        return "Line", 0.0

    curvature_score = float(np.mean(residuals) / bbox_diagonal)
    shape_type = "Line" if curvature_score < threshold else "Curve"
    return shape_type, curvature_score


def fit_line_endpoints(contour):
    """Fit a straight line to a contour; return its two extreme endpoints.

    Returns (p1, p2, direction) where p1/p2 are (x, y) pixel points and
    direction is the unit (vx, vy) of the line in image coordinates.
    """
    pts = contour.reshape(-1, 2).astype(np.float64)
    vx, vy, x0, y0 = cv.fitLine(pts, cv.DIST_L2, 0, 0.01, 0.01).ravel()
    direction = np.array([vx, vy])
    origin = np.array([x0, y0])

    # Project every point onto the fitted direction; min/max are the endpoints.
    t = (pts - origin) @ direction
    p1 = origin + direction * t.min()
    p2 = origin + direction * t.max()
    return p1, p2, direction


def pixel_to_camera(uv, camera_matrix, depth_mm):
    """Back-project a pixel to the camera frame (mm) under the planar assumption."""
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    u, v = uv
    x = (u - cx) * depth_mm / fx
    y = (v - cy) * depth_mm / fy
    return np.array([x, y, depth_mm])


def build_color_mask(image):
    """HSV mask for TARGET_COLOR (OR-ing all of its ranges), cleaned up."""
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    ranges = COLOR_RANGES[TARGET_COLOR]
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for lo, hi in ranges:
        mask |= cv.inRange(hsv, np.array(lo), np.array(hi))

    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5, 5))
    mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    return mask


def detect_lines(image):
    """Run the detection pipeline; return ALL detected lines (longest-first).

    Returns (lines, mask, edges). `lines` is a list of candidate dicts (same
    shape detect_line used to return), sorted by descending pixel length.
    Candidates classified as "Line" are listed first; if none classify as Line,
    every candidate is still returned so a lone curve can fall back.
    """
    mask = build_color_mask(image)

    gX = cv.Sobel(mask, ddepth=cv.CV_32F, dx=1, dy=0, ksize=3)
    gY = cv.Sobel(mask, ddepth=cv.CV_32F, dx=0, dy=1, ksize=3)
    combined = cv.addWeighted(cv.convertScaleAbs(gX), 0.5,
                              cv.convertScaleAbs(gY), 0.5, 0)
    _, edges = cv.threshold(combined, 50, 255, cv.THRESH_BINARY)

    contours, _ = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    candidates = []
    for contour in contours:
        # A thin line has tiny area; gate on perimeter instead.
        if cv.arcLength(contour, False) < 40:
            continue
        shape_type, curvature = is_line_or_curve(contour, threshold=0.15)
        p1, p2, direction = fit_line_endpoints(contour)
        length_px = float(np.linalg.norm(p2 - p1))
        candidates.append(
            {"contour": contour, "p1": p1, "p2": p2, "direction": direction,
             "length_px": length_px, "type": shape_type, "curvature": curvature})

    lines = [c for c in candidates if c["type"] == "Line"] or candidates
    lines.sort(key=lambda c: c["length_px"], reverse=True)
    return lines, mask, combined


def detect_line(image):
    """Single-line entry point (longest line), kept for backward compatibility."""
    lines, mask, edges = detect_lines(image)
    return (lines[0] if lines else None), mask, edges


def measure_line(line):
    """Compute length/orientation for a detected line. Returns a dict.

    Always includes pixel length and image/visual angles; adds camera-frame
    values (mm) when CAMERA_MATRIX and PLANE_DEPTH_MM are set.
    """
    p1, p2 = line["p1"], line["p2"]
    dx, dy = (p2 - p1)
    m = {
        "p1": p1, "p2": p2,
        "length_px": line["length_px"],
        "angle_img": float(np.degrees(np.arctan2(dy, dx))),      # pixel axes (y down)
        "angle_visual": float(np.degrees(np.arctan2(-dy, dx))),  # y up (on-screen)
        "type": line.get("type"),
    }
    if CAMERA_MATRIX is not None and PLANE_DEPTH_MM is not None:
        P1 = pixel_to_camera(p1, CAMERA_MATRIX, PLANE_DEPTH_MM)
        P2 = pixel_to_camera(p2, CAMERA_MATRIX, PLANE_DEPTH_MM)
        vec = P2 - P1
        length_mm = float(np.linalg.norm(vec))
        m.update({
            "P1": P1, "P2": P2,
            "length_mm": length_mm,
            "unit": vec / length_mm,
            "angle_cam": float(np.degrees(np.arctan2(vec[1], vec[0]))),
        })
    return m


def annotate(frame, line, m, index=None):
    """Draw the detected line outline + endpoints + label onto `frame` in place.

    `index` (1-based), when given, prefixes the label so multiple lines drawn on
    the same frame can be told apart.
    """
    cv.drawContours(frame, [line["contour"]], -1, (0, 255, 255), 1)

    pt1 = tuple(np.round(m["p1"]).astype(int))
    pt2 = tuple(np.round(m["p2"]).astype(int))
    cv.line(frame, pt1, pt2, (0, 255, 0), 2)
    cv.circle(frame, pt1, 5, (255, 0, 0), -1)
    cv.circle(frame, pt2, 5, (255, 0, 0), -1)

    mid = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)
    prefix = f"[{index}] " if index is not None else ""
    if "length_mm" in m:
        label = f"{prefix}{m['length_mm']:.0f}mm  {m['angle_cam']:.1f}deg"
    else:
        label = f"{prefix}{m['length_px']:.0f}px  {m['angle_visual']:.1f}deg"
    org = (mid[0] + 8, max(mid[1] - 8, 18))
    cv.putText(frame, label, org, cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
    cv.putText(frame, label, org, cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)


def measurement_summary(m):
    """Human-readable multi-line summary of a measurement dict (for the log)."""
    lines = [
        "=== Line Measurement ===",
        f"Endpoint 1 (px): ({m['p1'][0]:.1f}, {m['p1'][1]:.1f})",
        f"Endpoint 2 (px): ({m['p2'][0]:.1f}, {m['p2'][1]:.1f})",
        f"Length:          {m['length_px']:.1f} px",
        f"Orientation:     {m['angle_img']:.2f} deg from +X (image, y-down)",
        f"                 {m['angle_visual']:.2f} deg visual (y-up)",
    ]
    if "length_mm" in m:
        lines += [
            "--- camera frame (planar assumption) ---",
            f"Endpoint 1 (mm): ({m['P1'][0]:.1f}, {m['P1'][1]:.1f}, {m['P1'][2]:.1f})",
            f"Endpoint 2 (mm): ({m['P2'][0]:.1f}, {m['P2'][1]:.1f}, {m['P2'][2]:.1f})",
            f"Length:          {m['length_mm']:.1f} mm",
            f"Direction (cam): [{m['unit'][0]:.3f}, {m['unit'][1]:.3f}, {m['unit'][2]:.3f}]",
            f"Orientation:     {m['angle_cam']:.2f} deg from camera +X (XY plane)",
        ]
    else:
        lines.append("(Set plane depth for length in mm / camera frame.)")
    return "\n".join(lines)
