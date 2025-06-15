#!/usr/bin/env python3
import os
# Force OpenCV’s Qt backend to XCB (avoid Wayland plugin)
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np
import tempfile
from pathlib import Path
import depthai as dai

# ─── Configuration ───────────────────────────────────────────────
SOURCE_FOLDER   = "Test_Images"
FRAME_DELAY_SEC = 5.5                    # seconds per frame
TEMP_VIDEO_NAME = "folder_replay.mp4"

# Only consider rows y >= Y_MIN (ignore above, e.g. sky)
Y_MIN           = 250                   # pixels from top
# If midpoint jumps more than this between adjacent rows, clamp it
MAX_DELTA_X     = 30                     # pixels
# Smoothing factor for exponential filter (0 < α < 1)
SMOOTH_ALPHA    = 0.3

def make_video_from_folder(folder: str, output_file: str, delay_s: float):
    files = sorted(
        os.path.join(folder, f)
        for f in os.listdir(folder)
        if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp'))
    )
    if not files:
        raise RuntimeError(f"No images found in '{folder}'")
    first = cv2.imread(files[0]); h, w = first.shape[:2]
    fps = 1.0 / delay_s
    writer = cv2.VideoWriter(output_file,
                             cv2.VideoWriter_fourcc(*'mp4v'),
                             fps, (w, h))
    for path in files:
        img = cv2.imread(path)
        if img is None:
            print(f"[Warning] Skipping '{path}'"); continue
        if (img.shape[1], img.shape[0]) != (w, h):
            img = cv2.resize(img, (w, h))
        writer.write(img)
    writer.release()
    print(f"[Done] Created video {output_file} @ {fps:.2f} FPS")

def segment_track(frame: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([  0,   0,  80]); upper = np.array([180,  60, 255])
    mask = cv2.inRange(hsv, lower, upper)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask

def find_filtered_midpoints(mask: np.ndarray):
    h, w = mask.shape
    raw_pts = []
    # scan only from Y_MIN downwards
    for y in range(Y_MIN, h):
        xs = np.where(mask[y] > 0)[0]
        if xs.size:
            raw_pts.append(((xs[0] + xs[-1])//2, y))
    if not raw_pts:
        return []

    # Outlier clamping + exponential smoothing
    filtered = []
    prev_x = raw_pts[0][0]
    smooth_x = prev_x
    for x, y in raw_pts:
        # clamp |Δx| to MAX_DELTA_X
        dx = x - prev_x
        if abs(dx) > MAX_DELTA_X:
            x = prev_x + np.sign(dx)*MAX_DELTA_X
        # exponential smoothing
        smooth_x = SMOOTH_ALPHA * x + (1 - SMOOTH_ALPHA) * smooth_x
        ix = int(round(smooth_x))
        filtered.append((ix, y))
        prev_x = x
    return filtered

def main():
    # 1) Build MP4
    temp_mp4 = os.path.join(tempfile.gettempdir(), TEMP_VIDEO_NAME)
    make_video_from_folder(SOURCE_FOLDER, temp_mp4, FRAME_DELAY_SEC)

    # 2) Setup DepthAI replay
    with dai.Pipeline() as pipeline:
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(Path(temp_mp4))
        replay.setLoop(True)
        replay.setOutFrameType(dai.ImgFrame.Type.BGR888p)
        frames_q = replay.out.createOutputQueue()

        # 3) Start on OAK
        device = pipeline.start()

        # 4) Main loop: segment → filter midpoints → overlay
        while pipeline.isRunning():
            in_frame = frames_q.get()
            frame    = in_frame.getCvFrame()

            mask     = segment_track(frame)
            mid_pts  = find_filtered_midpoints(mask)

            # Print midpoints
            for x, y in mid_pts:
                print(f"({x},{y})")

            # Draw centerline
            disp = frame.copy()
            if len(mid_pts) >= 2:
                pts = np.array(mid_pts, dtype=np.int32).reshape(-1,1,2)
                cv2.polylines(disp, [pts], isClosed=False, color=(0,0,255), thickness=2)

            cv2.imshow("Filtered Track Midpoints", disp)
            if cv2.waitKey(1) == ord('q'):
                break

        device.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
