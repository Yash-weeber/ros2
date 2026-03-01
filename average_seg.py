# import cv2
# import numpy as np
# import os
# import time
# import csv
# import paramiko
# import threading

# # =========================
# # CONFIG & HARDCODED ROI
# # =========================
# CAMERA_INDEX = 0

# # Hardcoded ROI from your selection
# ROI_X, ROI_Y, ROI_W, ROI_H = 330, 200, 606, 286
# ZOOM_FACTOR = 1.0 

# # HSV thresholds
# H_MIN, H_MAX = 250, 65
# S_MIN, S_MAX = 0, 30
# V_MIN, V_MAX = 155, 211

# # =========================
# # FILE & TRANSFER SETTINGS
# # =========================
# LOG_DIR = "logs"
# CSV_PATH = os.path.join(LOG_DIR, "bead_counts.csv")

# SOL_USER = "abhapkar"
# SOL_HOST = "sol.asu.edu"
# SOL_REMOTE_DIR = "/home/abhapkar/COLOR"

# # Path to your specific private key (NO .pub extension)
# PRIVATE_KEY_PATH = r"F:\Robotics Automation system AI\PROf Hani\DETECTION\yrakesh1"

# # Timing config (in seconds)
# SCAN_DURATION = 6.0
# WAIT_DURATION = 60.0

# # Ensure log directory exists
# os.makedirs(LOG_DIR, exist_ok=True)

# # =========================
# # HELPER FUNCTIONS
# # =========================
# def segment_mask_from_frame(frame_bgr):
#     """Generates the segmented mask based on HSV limits."""
#     hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
#     hmin = int(H_MIN * 179 / 255)
#     hmax = int(H_MAX * 179 / 255)

#     if hmin <= hmax:
#         lower = np.array([hmin, S_MIN, V_MIN])
#         upper = np.array([hmax, S_MAX, V_MAX])
#         mask = cv2.inRange(hsv, lower, upper)
#     else:
#         mask1 = cv2.inRange(hsv, np.array([hmin, S_MIN, V_MIN]), np.array([179, S_MAX, V_MAX]))
#         mask2 = cv2.inRange(hsv, np.array([0, S_MIN, V_MIN]), np.array([hmax, S_MAX, V_MAX]))
#         mask = mask1 | mask2

#     return mask

# def sftp_transfer_worker():
#     """Handles the actual transfer via Paramiko silently in the background."""
#     try:
#         # Load the private key explicitly
#         key = paramiko.RSAKey.from_private_key_file(PRIVATE_KEY_PATH)
        
#         # Setup SSH client
#         ssh = paramiko.SSHClient()
#         ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        
#         # Connect to Sol completely passwordless
#         ssh.connect(hostname=SOL_HOST, username=SOL_USER, pkey=key)
        
#         # Open SFTP session and push the file
#         sftp = ssh.open_sftp()
#         remote_path = f"{SOL_REMOTE_DIR}/bead_counts.csv"
#         sftp.put(CSV_PATH, remote_path)
        
#         # Clean up
#         sftp.close()
#         ssh.close()
#         print(f"\n[SUCCESS] Silently transferred CSV to {SOL_HOST}")
        
#     except Exception as e:
#         print(f"\n[ERROR] Transfer failed: {e}")

# def save_and_transfer(iteration, max_balls):
#     """Overwrites the CSV and starts a background thread for transfer."""
#     # 1. Overwrite CSV
#     with open(CSV_PATH, mode='w', newline='') as f:
#         writer = cv2.csv.writer(f) if hasattr(cv2, 'csv') else csv.writer(f)
#         writer.writerow(['iter', 'total_balls'])
#         writer.writerow([iteration, max_balls])
    
#     print(f"\n[ITERATION {iteration}] Saved max count: {max_balls} to {CSV_PATH}")

#     # 2. Transfer via Paramiko SFTP in a background thread
#     transfer_thread = threading.Thread(target=sftp_transfer_worker)
#     transfer_thread.start()

# # =========================
# # MAIN LOOP
# # =========================
# def main():
#     cap = cv2.VideoCapture(CAMERA_INDEX)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#     if not cap.isOpened():
#         print("Camera error")
#         return

#     # State variables
#     current_iter = 1
#     state = "SCANNING"  # Can be "SCANNING" or "WAITING"
#     state_start_time = time.time()
#     max_balls_in_scan = 0

#     print(f"\n[INFO] Starting automated capture. Hardcoded ROI loaded.")
#     print("Controls: Press 'q' to quit.\n")

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break

#         # Crop to the hardcoded ROI
#         x2, y2 = ROI_X + ROI_W, ROI_Y + ROI_H
#         inference_frame = frame[ROI_Y:y2, ROI_X:x2]

#         # Draw ROI on the display frame
#         cv2.rectangle(frame, (ROI_X, ROI_Y), (x2, y2), (0, 255, 0), 2)

#         # Process segmentation & counting
#         mask = segment_mask_from_frame(inference_frame)
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         current_blob_count = len(contours)

#         # State Machine Logic
#         elapsed_time = time.time() - state_start_time

#         if state == "SCANNING":
#             # Track the absolute maximum found in this window
#             if current_blob_count > max_balls_in_scan:
#                 max_balls_in_scan = current_blob_count
            
#             time_left = max(0, SCAN_DURATION - elapsed_time)
#             status_text = f"SCANNING (Iter {current_iter}) - {time_left:.1f}s left"
#             cv2.putText(frame, f"Max Balls Seen: {max_balls_in_scan}", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

#             if elapsed_time >= SCAN_DURATION:
#                 # Time is up! Save, send, and switch states.
#                 save_and_transfer(current_iter, max_balls_in_scan)
#                 state = "WAITING"
#                 state_start_time = time.time()

#         elif state == "WAITING":
#             time_left = max(0, WAIT_DURATION - elapsed_time)
#             status_text = f"WAITING - {time_left:.1f}s left"
#             cv2.putText(frame, f"Last Max Saved: {max_balls_in_scan}", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

#             if elapsed_time >= WAIT_DURATION:
#                 # Waiting done! Switch back to scanning.
#                 state = "SCANNING"
#                 state_start_time = time.time()
#                 current_iter += 1
#                 max_balls_in_scan = 0  # Reset for the new scan

#         # Overlay generic info
#         cv2.putText(frame, status_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
#         cv2.putText(frame, f"Current Blobs: {current_blob_count}", (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

#         # Show windows
#         result_vis = cv2.bitwise_and(inference_frame, inference_frame, mask=mask)
#         cv2.imshow("Live Feed", frame)
#         cv2.imshow("Segmented (ROI)", result_vis)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()



















# ----------------------------------------------------------------------------------------------------------------------------------------
# old segment code 



# import cv2
# import numpy as np
# import os
# import time
# import csv
# import paramiko
# import threading

# # =========================
# # CONFIG & HARDCODED ROI (Custom 4-Point Polygon)
# # =========================
# CAMERA_INDEX = 0

# # Define the exact X,Y coordinates for the 4 corners of your scanning area.
# # You can move any of these points to stretch or shrink any side indepsendently.
# PT_TOP_LEFT     = (435, 140)
# PT_TOP_RIGHT    = (996, 140)
# PT_BOTTOM_RIGHT = (996, 565)
# PT_BOTTOM_LEFT  = (400, 555)

# # Group them into a NumPy array for OpenCV to draw the polygon
# ROI_POLYGON = np.array([
#     PT_TOP_LEFT,
#     PT_TOP_RIGHT,
#     PT_BOTTOM_RIGHT,
#     PT_BOTTOM_LEFT
# ], np.int32)

# ZOOM_FACTOR = 1.0 

# # HSV thresholds
# H_MIN, H_MAX = 250, 65
# S_MIN, S_MAX = 0, 30
# V_MIN, V_MAX = 155, 211

# # =========================
# # FILE & TRANSFER SETTINGS
# # =========================
# LOG_DIR = "logs"
# CSV_PATH = os.path.join(LOG_DIR, "bead_counts.csv")

# SOL_USER = "abhapkar"
# SOL_HOST = "sol.asu.edu"
# SOL_REMOTE_DIR = "/home/abhapkar/COLOR"

# PRIVATE_KEY_PATH = r"F:\Robotics Automation system AI\PROf Hani\DETECTION\yrakesh1"

# SCAN_DURATION = 6.0
# WAIT_DURATION = 60.0

# os.makedirs(LOG_DIR, exist_ok=True)

# # =========================
# # HELPER FUNCTIONS
# # =========================
# def segment_mask_from_frame(frame_bgr):
#     """Generates the segmented mask based on HSV limits."""
#     hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
#     hmin = int(H_MIN * 179 / 255)
#     hmax = int(H_MAX * 179 / 255)

#     if hmin <= hmax:
#         lower = np.array([hmin, S_MIN, V_MIN])
#         upper = np.array([hmax, S_MAX, V_MAX])
#         mask = cv2.inRange(hsv, lower, upper)
#     else:
#         mask1 = cv2.inRange(hsv, np.array([hmin, S_MIN, V_MIN]), np.array([179, S_MAX, V_MAX]))
#         mask2 = cv2.inRange(hsv, np.array([0, S_MIN, V_MIN]), np.array([hmax, S_MAX, V_MAX]))
#         mask = mask1 | mask2

#     return mask

# def sftp_transfer_worker():
#     """Handles the actual transfer via Paramiko silently in the background."""
#     try:
#         key = paramiko.RSAKey.from_private_key_file(PRIVATE_KEY_PATH)
#         ssh = paramiko.SSHClient()
#         ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
#         ssh.connect(hostname=SOL_HOST, username=SOL_USER, pkey=key)
#         sftp = ssh.open_sftp()
#         remote_path = f"{SOL_REMOTE_DIR}/bead_counts.csv"
#         sftp.put(CSV_PATH, remote_path)
#         sftp.close()
#         ssh.close()
#         print(f"\n[SUCCESS] Silently transferred CSV to {SOL_HOST}")
#     except Exception as e:
#         print(f"\n[ERROR] Transfer failed: {e}")

# def save_and_transfer(iteration, max_balls):
#     """Overwrites the CSV and starts a background thread for transfer."""
#     with open(CSV_PATH, mode='w', newline='') as f:
#         writer = cv2.csv.writer(f) if hasattr(cv2, 'csv') else csv.writer(f)
#         writer.writerow(['iter', 'total_balls'])
#         writer.writerow([iteration, max_balls])
    
#     print(f"\n[ITERATION {iteration}] Saved max count: {max_balls} to {CSV_PATH}")

#     transfer_thread = threading.Thread(target=sftp_transfer_worker)
#     transfer_thread.start()

# # =========================
# # MAIN LOOP
# # =========================
# def main():
#     cap = cv2.VideoCapture(CAMERA_INDEX)
#     cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#     cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#     if not cap.isOpened():
#         print("Camera error")
#         return

#     current_iter = 1
#     state = "SCANNING"  
#     state_start_time = time.time()
#     max_balls_in_scan = 0
    
#     sum_balls_in_scan = 0
#     frame_count_in_scan = 0
#     avg_balls_in_scan = 0

#     print(f"\n[INFO] Starting automated capture. Custom Polygon ROI loaded.")
#     print("Controls: Press 'q' to quit.\n")

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break

#         # 1. Create a blank black image exactly the same size as the camera frame
#         polygon_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        
#         # 2. Draw your custom white polygon onto that black image
#         cv2.fillPoly(polygon_mask, [ROI_POLYGON], 255)
        
#         # 3. Apply the mask to the live camera feed so everything outside the polygon goes pitch black
#         inference_frame = cv2.bitwise_and(frame, frame, mask=polygon_mask)

#         # Draw the green outline of your custom ROI on the display frame so you can see it
#         cv2.polylines(frame, [ROI_POLYGON], isClosed=True, color=(0, 255, 0), thickness=2)

#         # Process segmentation & counting (only operates on the visible area inside the polygon)
#         mask = segment_mask_from_frame(inference_frame)
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         current_blob_count = len(contours)

#         # State Machine Logic
#         elapsed_time = time.time() - state_start_time

#         # if state == "SCANNING":
#         #     if current_blob_count > max_balls_in_scan:
#         #         max_balls_in_scan = current_blob_count
            
#         #     time_left = max(0, SCAN_DURATION - elapsed_time)
#         #     status_text = f"SCANNING (Iter {current_iter}) - {time_left:.1f}s left"
#         #     cv2.putText(frame, f"Max Balls Seen: {max_balls_in_scan}", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

#         #     if elapsed_time >= SCAN_DURATION:
#         #         save_and_transfer(current_iter, max_balls_in_scan)
#         #         state = "WAITING"
#         #         state_start_time = time.time()
    
#         # elif state == "WAITING":
#         #     time_left = max(0, WAIT_DURATION - elapsed_time)
#         #     status_text = f"WAITING - {time_left:.1f}s left"
#         #     cv2.putText(frame, f"Last Max Saved: {max_balls_in_scan}", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

#         #     if elapsed_time >= WAIT_DURATION:
#         #         state = "SCANNING"
#         #         state_start_time = time.time()
#         #         current_iter += 1
#         #         max_balls_in_scan = 0  

#         if state == "SCANNING":
#             # Track maximum
#             if current_blob_count > max_balls_in_scan:
#                 max_balls_in_scan = current_blob_count

#             # Track average
#             sum_balls_in_scan += current_blob_count
#             frame_count_in_scan += 1

#             if frame_count_in_scan > 0:
#                 avg_balls_in_scan = sum_balls_in_scan / frame_count_in_scan

#             time_left = max(0, SCAN_DURATION - elapsed_time)
#             status_text = f"SCANNING (Iter {current_iter}) - {time_left:.1f}s left"

#             cv2.putText(frame, f"Max Balls Seen: {max_balls_in_scan}", 
#                         (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

#             cv2.putText(frame, f"Avg Balls: {avg_balls_in_scan:.2f}", 
#                         (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

#     if elapsed_time >= SCAN_DURATION:
#         save_and_transfer(current_iter, max_balls_in_scan)
#         state = "WAITING"
#         state_start_time = time.time()
#         if elapsed_time >= WAIT_DURATION:
#             state = "SCANNING"
#             state_start_time = time.time()
#             current_iter += 1

#             max_balls_in_scan = 0
#             sum_balls_in_scan = 0
#             frame_count_in_scan = 0
#             avg_balls_in_scan = 0

#         # Overlay generic info
#         cv2.putText(frame, status_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
#         cv2.putText(frame, f"Current Blobs: {current_blob_count}", (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

#         # Show windows
#         result_vis = cv2.bitwise_and(inference_frame, inference_frame, mask=mask)
#         cv2.imshow("Live Feed", frame)
#         cv2.imshow("Segmented (Custom ROI)", result_vis)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()

# if __name__ == "__main__":
#     main()



# ======================================================================================================================
# new update code with avg 



import cv2
import numpy as np
import os
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
PT_BOTTOM_LEFT  = (400, 555)

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
WAIT_DURATION = 60.0

os.makedirs(LOG_DIR, exist_ok=True)

# =========================
# HELPER FUNCTIONS
# =========================
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
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

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
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
