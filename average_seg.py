import cv2
import numpy as np
# import os
import os

# Silence GStreamer log noise before importing cv2
os.environ["GST_DEBUG"] = "0"                        # Disable all GStreamer debug output
import time
import csv
import paramiko
import threading

# =========================
# CONFIG & HARDCODED ROI (Custom 4-Point Polygon)
# =========================
CAMERA_INDEX = 0

PT_TOP_LEFT     = (435, 140)
PT_TOP_RIGHT    = (996, 140)
PT_BOTTOM_RIGHT = (996, 565)
PT_BOTTOM_LEFT  = (400, 550)

ROI_POLYGON = np.array([
    PT_TOP_LEFT,
    PT_TOP_RIGHT,
    PT_BOTTOM_RIGHT,
    PT_BOTTOM_LEFT
], np.int32)

ZOOM_FACTOR = 1.0 

# HSV thresholds
H_MIN, H_MAX = 250, 65
S_MIN, S_MAX = 0, 30
V_MIN, V_MAX = 155, 211

# =========================
# FILE & TRANSFER SETTINGS
# =========================
LOG_DIR = "logs"
CSV_PATH = os.path.join(LOG_DIR, "bead_counts.csv")

SOL_USER = "abhapkar"
SOL_HOST = "sol.asu.edu"
SOL_REMOTE_DIR = "/home/abhapkar/COLOR"

PRIVATE_KEY_PATH = r"F:\Robotics Automation system AI\PROf Hani\DETECTION\yrakesh1"

SCAN_DURATION = 6.0
WAIT_DURATION = 10.0

os.makedirs(LOG_DIR, exist_ok=True)

# =========================
# HELPER FUNCTIONS
# =========================

def open_camera_gst(device="/dev/video0", width=1280, height=720, fps=30):
    # Try MJPEG first (common for USB webcams)
    pipeline_mjpeg = (
        f"v4l2src device={device} ! "
        f"image/jpeg,width={width},height={height},framerate={fps}/1 ! "
        f"jpegdec ! videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )

    cap = cv2.VideoCapture(pipeline_mjpeg, cv2.CAP_GSTREAMER)
    if cap.isOpened():
        return cap

    # Fallback: raw YUY2
    pipeline_yuy2 = (
        f"v4l2src device={device} ! "
        f"video/x-raw,format=YUY2,width={width},height={height},framerate={fps}/1 ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink drop=true max-buffers=1 sync=false"
    )
    cap = cv2.VideoCapture(pipeline_yuy2, cv2.CAP_GSTREAMER)
    return cap

def segment_mask_from_frame(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    hmin = int(H_MIN * 179 / 255)
    hmax = int(H_MAX * 179 / 255)

    if hmin <= hmax:
        lower = np.array([hmin, S_MIN, V_MIN])
        upper = np.array([hmax, S_MAX, V_MAX])
        mask = cv2.inRange(hsv, lower, upper)
    else:
        mask1 = cv2.inRange(hsv, np.array([hmin, S_MIN, V_MIN]), np.array([179, S_MAX, V_MAX]))
        mask2 = cv2.inRange(hsv, np.array([0, S_MIN, V_MIN]), np.array([hmax, S_MAX, V_MAX]))
        mask = mask1 | mask2

    return mask

def sftp_transfer_worker():
    try:
        key = paramiko.RSAKey.from_private_key_file(PRIVATE_KEY_PATH)
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname=SOL_HOST, username=SOL_USER, pkey=key)
        sftp = ssh.open_sftp()
        remote_path = f"{SOL_REMOTE_DIR}/bead_counts.csv"
        sftp.put(CSV_PATH, remote_path)
        sftp.close()
        ssh.close()
        print(f"\n[SUCCESS] Silently transferred CSV to {SOL_HOST}")
    except Exception as e:
        print(f"\n[ERROR] Transfer failed: {e}")

def save_and_transfer(iteration, max_balls):
    with open(CSV_PATH, mode='w', newline='') as f:
        writer = cv2.csv.writer(f) if hasattr(cv2, 'csv') else csv.writer(f)
        writer.writerow(['iter', 'total_balls'])
        writer.writerow([iteration, max_balls])
    
    print(f"\n[ITERATION {iteration}] Saved max count: {max_balls} to {CSV_PATH}")

    transfer_thread = threading.Thread(target=sftp_transfer_worker)
    transfer_thread.start()

# =========================
# MAIN LOOP
# =========================
def main():
    cap = open_camera_gst("/dev/video0", 1280, 720, 30)
    if not cap.isOpened():
        print("Camera error")
        return

    current_iter = 1
    state = "SCANNING"
    state_start_time = time.time()

    max_balls_in_scan = 0
    sum_balls_in_scan = 0
    frame_count_in_scan = 0
    avg_balls_in_scan = 0

    print(f"\n[INFO] Starting automated capture. Custom Polygon ROI loaded.")
    print("Controls: Press 'q' to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        polygon_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.fillPoly(polygon_mask, [ROI_POLYGON], 255)
        inference_frame = cv2.bitwise_and(frame, frame, mask=polygon_mask)
        cv2.polylines(frame, [ROI_POLYGON], True, (0, 255, 0), 2)

        mask = segment_mask_from_frame(inference_frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_import os
os.environ["GST_DEBUG"] = "0"  # optional: reduce GStreamer noise

import cv2
import numpy as np
from collections import deque

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
MIN_BLOB_AREA = 3
MAX_BLOB_AREA = 2500
COUNT_SMOOTH_WINDOW = 15

# Split touching blobs
SPLIT_TRIGGER_AREA = 12
PEAK_REL_THRESH = 0.28
PEAK_NMS_KERNEL = 3
MIN_PEAK_AREA = 1

# Hybrid counting (new)
SINGLE_AREA_MIN = 3
SINGLE_AREA_MAX = 80
SINGLE_BALL_AREA_FALLBACK = 12.0

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

    # Otsu threshold from ROI-only pixels
    roi_vals = blackhat[roi_mask > 0]
    if roi_vals.size == 0:
        return np.zeros_like(gray, dtype=np.uint8)

    thr, _ = cv2.threshold(roi_vals, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
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
    cap = open_camera_gst()
    if not cap.isOpened():
        print("Camera error")
        return

    # Warm-up
    for _ in range(30):
        cap.read()

    count_hist = deque(maxlen=COUNT_SMOOTH_WINDOW)

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        roi_mask = build_roi_mask(frame.shape[:2])
        raw_mask = segment_black_balls(frame, roi_mask)
        raw_count, filtered_mask = count_components(raw_mask)

        # Stabilized count
        count_hist.append(raw_count)
        stable_count = int(np.median(count_hist))

        # Draw ROI and text
        cv2.polylines(frame, [ROI_POLYGON], True, (0, 255, 0), 2)
        cv2.putText(frame, f"Ball Count (ROI): {stable_count}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
        cv2.putText(frame, f"Mask px: {int(cv2.countNonZero(filtered_mask))}", (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 255, 200), 2)

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
    main()APPROX_SIMPLE)
        current_blob_count = len(contours)

        elapsed_time = time.time() - state_start_time

        # OLD STATE MACHINE (KEPT EXACTLY AS YOU WROTE IT)
        # if state == "SCANNING":
        #     if current_blob_count > max_balls_in_scan:
        #         max_balls_in_scan = current_blob_count
        #     
        #     time_left = max(0, SCAN_DURATION - elapsed_time)
        #     status_text = f"SCANNING (Iter {current_iter}) - {time_left:.1f}s left"
        #     cv2.putText(frame, f"Max Balls Seen: {max_balls_in_scan}", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        #
        #     if elapsed_time >= SCAN_DURATION:
        #         save_and_transfer(current_iter, max_balls_in_scan)
        #         state = "WAITING"
        #         state_start_time = time.time()
        #
        # elif state == "WAITING":
        #     time_left = max(0, WAIT_DURATION - elapsed_time)
        #     status_text = f"WAITING - {time_left:.1f}s left"
        #     cv2.putText(frame, f"Last Max Saved: {max_balls_in_scan}", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        #
        #     if elapsed_time >= WAIT_DURATION:
        #         state = "SCANNING"
        #         state_start_time = time.time()
        #         current_iter += 1
        #         max_balls_in_scan = 0  

        # NEW FIXED STATE MACHINE
        if state == "SCANNING":

            if current_blob_count > max_balls_in_scan:
                max_balls_in_scan = current_blob_count

            sum_balls_in_scan += current_blob_count
            frame_count_in_scan += 1
            avg_balls_in_scan = (sum_balls_in_scan / frame_count_in_scan) / 2

            time_left = max(0, SCAN_DURATION - elapsed_time)
            status_text = f"SCANNING (Iter {current_iter}) - {time_left:.1f}s left"

            cv2.putText(frame, f"Max Balls Seen: {max_balls_in_scan}",
                        (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.putText(frame, f"Avg Balls: {avg_balls_in_scan:.2f}",
                        (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            if elapsed_time >= SCAN_DURATION:
                save_and_transfer(current_iter, max_balls_in_scan)
                state = "WAITING"
                state_start_time = time.time()

        elif state == "WAITING":

            time_left = max(0, WAIT_DURATION - elapsed_time)
            status_text = f"WAITING - {time_left:.1f}s left"

            cv2.putText(frame, f"Last Max Saved: {max_balls_in_scan}",
                        (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

            cv2.putText(frame, f"Last Avg: {avg_balls_in_scan:.2f}",
                        (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            if elapsed_time >= WAIT_DURATION:
                state = "SCANNING"
                state_start_time = time.time()
                current_iter += 1

                max_balls_in_scan = 0
                sum_balls_in_scan = 0
                frame_count_in_scan = 0
                avg_balls_in_scan = 0

        cv2.putText(frame, status_text, (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.putText(frame, f"Current Blobs: {current_blob_count}",
                    (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        result_vis = cv2.bitwise_and(inference_frame, inference_frame, mask=mask)
        cv2.imshow("Live Feed", frame)
        cv2.imshow("Segmented (Custom ROI)", result_vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()


#-------METHOD TO RUN----
# pip install paramiko
# ssh-keygen -t rsa
# scp KEYNAME.pub abhapkar@sol.asu.edu:~/temp_key.pub
# ssh abhapkar@sol.asu.edu "mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat ~/temp_key.pub >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && rm ~/temp_key.pub"
