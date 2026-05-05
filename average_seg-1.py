import os
os.environ["GST_DEBUG"] = "0"  # optional: reduce GStreamer noise

import cv2
import numpy as np
from collections import deque
import subprocess
import time

# =========================
# CONFIG
# =========================
DEVICE = "/dev/video0"
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FPS = 30

# Work area polygon (your green box)
PT_TOP_LEFT = (425, 175)
PT_TOP_RIGHT = (996, 185)
PT_BOTTOM_RIGHT = (996, 525)
PT_BOTTOM_LEFT = (400, 525)

ROI_POLYGON = np.array(
    [PT_TOP_LEFT, PT_TOP_RIGHT, PT_BOTTOM_RIGHT, PT_BOTTOM_LEFT],
    dtype=np.int32
)

# Detection tuning for black beads
BLACKHAT_KERNEL = 11
OPEN_KERNEL = 3
CLOSE_KERNEL = 3
MIN_BLOB_AREA = 1
MAX_BLOB_AREA = 2500
COUNT_SMOOTH_WINDOW = 15
ROLLING_WINDOW_SEC = 30.0

# Split touching blobs
SPLIT_TRIGGER_AREA = 12
PEAK_REL_THRESH = 0.2
PEAK_NMS_KERNEL = 3
MIN_PEAK_AREA = 1

# Hybrid counting (new)
SINGLE_AREA_MIN = 1
SINGLE_AREA_MAX = 80
SINGLE_BALL_AREA_FALLBACK = 12.0

# Threshold hardening (reduce background noise hits)
THRESH_OFFSET = 8          # increase to 10-14 if still noisy
THRESH_MIN = 12            # absolute minimum threshold
THRESH_PERCENTILE = 98.0   # ROI percentile guard

# =========================
# CAMERA
# =========================
def open_camera_gst(device=DEVICE, width=FRAME_WIDTH, height=FRAME_HEIGHT, fps=FPS):
    """Open camera using GStreamer (MJPEG first, then YUY2 fallback)."""
    pipeline_mjpeg = (
        f"v4l2src device={device} ! "
        f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
        f"jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )
    cap = cv2.VideoCapture(pipeline_mjpeg, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        return cap

    pipeline_yuy2 = (
        f"v4l2src device={device} ! "
        f"video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )
    cap = cv2.VideoCapture(pipeline_yuy2, cv2.CAP_GSTREAMER)
    return cap

def apply_camera_controls(device=DEVICE):
    """Apply stable manual camera settings for segmentation."""
    cmds = [
        ["v4l2-ctl", "-d", device, "-c", "focus_auto=0"],
        ["v4l2-ctl", "-d", device, "-c", "focus_absolute=120"],
        ["v4l2-ctl", "-d", device, "-c", "exposure_auto=1"],
        ["v4l2-ctl", "-d", device, "-c", "exposure_time_absolute=50"],
        ["v4l2-ctl", "-d", device, "-c", "brightness=90"],
        ["v4l2-ctl", "-d", device, "-c", "contrast=60"],
        ["v4l2-ctl", "-d", device, "-c", "white_balance_temperature_auto=0"],
        ["v4l2-ctl", "-d", device, "-c", "white_balance_temperature=4500"],
    ]
    for c in cmds:
        subprocess.run(c, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# =========================
# SEGMENTATION / COUNTING
# =========================
def build_roi_mask(shape_hw):
    """Create binary mask for polygon ROI."""
    mask = np.zeros(shape_hw, dtype=np.uint8)
    cv2.fillPoly(mask, [ROI_POLYGON], 255)
    return mask


def segment_black_balls(frame_bgr, roi_mask):
    """
    Segment black balls on bright background.
    Threshold is computed from ROI pixels only.
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

    # Improve local contrast + denoise
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    gray = clahe.apply(gray)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # Emphasize dark small objects on bright background
    k_bh = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (BLACKHAT_KERNEL, BLACKHAT_KERNEL))
    blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, k_bh)

    # Otsu + upward bias + percentile floor
    roi_vals = blackhat[roi_mask > 0]
    if roi_vals.size == 0:
        return np.zeros_like(gray, dtype=np.uint8)

    otsu_thr, _ = cv2.threshold(roi_vals, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    p_thr = float(np.percentile(roi_vals, THRESH_PERCENTILE))
    thr = int(max(otsu_thr + THRESH_OFFSET, p_thr, THRESH_MIN))

    mask = np.zeros_like(gray, dtype=np.uint8)
    mask[blackhat >= thr] = 255

    # Keep only ROI
    mask = cv2.bitwise_and(mask, roi_mask)

    # Morphological cleanup
    k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_KERNEL, OPEN_KERNEL))
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_KERNEL, CLOSE_KERNEL))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close, iterations=1)

    return mask


def _count_peaks_in_component(component_mask):
    """
    Estimate count inside one merged component via distance-transform peaks.
    """
    dist = cv2.distanceTransform(component_mask, cv2.DIST_L2, 5)
    dmax = float(dist.max())
    if dmax <= 0.0:
        return 1

    dist = cv2.GaussianBlur(dist, (0, 0), 0.8)

    kernel = np.ones((PEAK_NMS_KERNEL, PEAK_NMS_KERNEL), np.uint8)
    dist_dil = cv2.dilate(dist, kernel)

    # local maxima + relative threshold
    peak_map = (dist >= (dist_dil - 1e-6)) & (dist >= (PEAK_REL_THRESH * dmax))
    peak_map = (peak_map.astype(np.uint8) * 255)

    n, _, stats, _ = cv2.connectedComponentsWithStats(peak_map, connectivity=8)

    valid_peaks = 0
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= MIN_PEAK_AREA:
            valid_peaks += 1

    return max(1, valid_peaks)


def _estimate_single_ball_area(stats):
    """
    Estimate single-ball pixel area from likely isolated components.
    """
    candidates = []
    for i in range(1, stats.shape[0]):
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < SINGLE_AREA_MIN or area > SINGLE_AREA_MAX:
            continue

        w = int(stats[i, cv2.CC_STAT_WIDTH])
        h = int(stats[i, cv2.CC_STAT_HEIGHT])
        if h == 0:
            continue

        aspect = w / float(h)
        if 0.45 <= aspect <= 2.2:
            candidates.append(area)

    if not candidates:
        return SINGLE_BALL_AREA_FALLBACK

    return float(np.median(candidates))


def count_components(mask):
    """
    Count balls with merged-blob splitting (hybrid peak + area estimator).
    Returns:
      - count: int
      - filtered_mask: binary mask of accepted components
    """
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    filtered = np.zeros_like(mask)
    count = 0

    single_ball_area = _estimate_single_ball_area(stats)

    for i in range(1, num_labels):  # skip background
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area < MIN_BLOB_AREA or area > MAX_BLOB_AREA:
            continue

        component = np.zeros_like(mask)
        component[labels == i] = 255

        if area >= SPLIT_TRIGGER_AREA:
            peak_est = _count_peaks_in_component(component)
            area_est = max(1, int(round(area / max(1.0, single_ball_area))))
            est = max(peak_est, area_est)  # anti-undercount rule for merged clusters
            count += est
        else:
            count += 1

        filtered[labels == i] = 255

    return count, filtered


# =========================
# MAIN
# =========================
def main():
    apply_camera_controls("/dev/video0")
    cap = open_camera_gst()
    if not cap.isOpened():
        print("Camera error")
        return

    for _ in range(30):
        cap.read()

    count_hist = deque(maxlen=COUNT_SMOOTH_WINDOW)

    # Fixed 60s window stats (resets every 60s)
    window_start = time.monotonic()
    window_sum = 0
    window_n = 0
    window_min = None
    window_max = None

    # Last completed 60s window (for display)
    last_avg = 0.0
    last_min = 0
    last_max = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        roi_mask = build_roi_mask(frame.shape[:2])
        raw_mask = segment_black_balls(frame, roi_mask)
        raw_count, filtered_mask = count_components(raw_mask)

        count_hist.append(raw_count)
        stable_count = int(np.median(count_hist))

        # Update current 60s block
        window_sum += stable_count
        window_n += 1
        window_min = stable_count if window_min is None else min(window_min, stable_count)
        window_max = stable_count if window_max is None else max(window_max, stable_count)

        now = time.monotonic()
        elapsed = now - window_start

        # Close and reset every 60s
        if elapsed >= ROLLING_WINDOW_SEC:
            last_avg = (window_sum / window_n) if window_n else 0.0
            last_min = window_min if window_min is not None else 0
            last_max = window_max if window_max is not None else 0

            window_start = now
            window_sum = 0
            window_n = 0
            window_min = None
            window_max = None

        secs_left = max(0.0, ROLLING_WINDOW_SEC - (time.monotonic() - window_start))

        # Draw ROI and text
        cv2.polylines(frame, [ROI_POLYGON], True, (0, 255, 0), 2)
        cv2.putText(frame, f"Ball Count (ROI): {stable_count}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
        cv2.putText(frame, f"Mask px: {int(cv2.countNonZero(filtered_mask))}", (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 200), 2)
        cv2.putText(frame, f"Last 60s Avg/Min/Max: {last_avg:.2f} / {last_min} / {last_max}", (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, f"Window refresh in: {secs_left:4.1f}s", (20, 145),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Visualization
        seg_vis = cv2.bitwise_and(frame, frame, mask=filtered_mask)
        cv2.imshow("Live Feed", frame)
        cv2.imshow("Detected Balls (Inside ROI Only)", seg_vis)
        cv2.imshow("Binary Mask", filtered_mask)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()