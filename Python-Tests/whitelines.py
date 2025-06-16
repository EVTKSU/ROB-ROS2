#!/usr/bin/env python3
import os
# Force OpenCV’s Qt backend to XCB (avoid Wayland plugin issues)
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np
import tempfile
from pathlib import Path
import depthai as dai
import time

# ─── Configuration ───────────────────────────────────────────────
SOURCE_FOLDER   = "Test_Images"
FRAME_DELAY_SEC = 5.5               # seconds per frame
TEMP_VIDEO_NAME = "folder_replay.mp4"
Y_MIN           = 250               # ignore rows above this pixel
MAX_DELTA_X     = 30                # clamp per-row midpoint jumps
SMOOTH_ALPHA    = 0.3               # for exponential midpoint smoothing

# White detection in HSV (tuned for bright white lines)
WHITE_LOWER = np.array([  0,   0, 200])
WHITE_UPPER = np.array([180,  50, 255])

def make_video_from_folder(folder, out_file, delay_s):
    """Turn sorted images into an MP4 at 1/delay_s FPS."""
    files = sorted(
        os.path.join(folder, f)
        for f in os.listdir(folder)
        if f.lower().endswith(('.png','.jpg','.jpeg','.bmp'))
    )
    if not files:
        raise RuntimeError(f"No images in '{folder}'")
    first = cv2.imread(files[0])
    h, w = first.shape[:2]
    fps  = 1.0 / delay_s
    writer = cv2.VideoWriter(
        out_file, cv2.VideoWriter_fourcc(*'mp4v'),
        fps, (w, h)
    )
    for p in files:
        img = cv2.imread(p)
        if img is None:
            print(f"[Warn] Skipping '{p}'"); continue
        if (img.shape[1], img.shape[0]) != (w, h):
            img = cv2.resize(img, (w, h))
        writer.write(img)
    writer.release()
    print(f"[Done] Video → {out_file} @ {fps:.2f} FPS")

def segment_track(frame):
    """HSV threshold + morphology to isolate asphalt (gray) track area."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0,0,80]), np.array([180,60,255]))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask

def find_filtered_midpoints(mask):
    """Scan rows ≥ Y_MIN, clamp jumps, and apply exponential smoothing."""
    h, w = mask.shape
    raw = []
    for y in range(Y_MIN, h):
        xs = np.where(mask[y]>0)[0]
        if xs.size:
            raw.append(((xs[0]+xs[-1])//2, y))
    if not raw:
        return []
    filtered, prev_x = [], raw[0][0]
    smooth_x = prev_x
    for x,y in raw:
        dx = x - prev_x
        if abs(dx)>MAX_DELTA_X:
            x = prev_x + np.sign(dx)*MAX_DELTA_X
        smooth_x = SMOOTH_ALPHA*x + (1-SMOOTH_ALPHA)*smooth_x
        ix = int(round(smooth_x))
        filtered.append((ix,y))
        prev_x = x
    return filtered

def detect_white_edge_slopes(frame):
    """
    Threshold for white lines, run Hough, then compute average slope
    (dy/dx) for left vs. right based on x-midpoint of each segment.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white_mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, k)
    lines = cv2.HoughLinesP(
        white_mask, 1, np.pi/180, threshold=50,
        minLineLength=30, maxLineGap=15
    )
    h,w = frame.shape[:2]
    left_slopes, right_slopes = [], []
    if lines is not None:
        for x1,y1,x2,y2 in lines.reshape(-1,4):
            # avoid vertical segments
            if x2==x1: continue
            slope = (y2-y1)/(x2-x1)
            mx = (x1+x2)/2
            if mx < w/2:
                left_slopes.append(slope)
            else:
                right_slopes.append(slope)
    avg_left  = np.mean(left_slopes)  if left_slopes  else None
    avg_right = np.mean(right_slopes) if right_slopes else None
    return white_mask, lines, avg_left, avg_right

def main():
    # 1) Build the MP4 from folder
    temp_mp4 = os.path.join(tempfile.gettempdir(), TEMP_VIDEO_NAME)
    make_video_from_folder(SOURCE_FOLDER, temp_mp4, FRAME_DELAY_SEC)

    # 2) DepthAI pipeline for replay
    with dai.Pipeline() as pipeline:
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(Path(temp_mp4))
        replay.setLoop(True)
        replay.setOutFrameType(dai.ImgFrame.Type.BGR888p)
        q = replay.out.createOutputQueue()
        device = pipeline.start()

        # 3) Main loop
        while pipeline.isRunning():
            in_fr = q.get()
            frame = in_fr.getCvFrame()

            # a) track segmentation + midpoints
            mask    = segment_track(frame)
            pts     = find_filtered_midpoints(mask)

            # b) white edge detection + slope calc
            wmask, lines, sl_left, sl_right = detect_white_edge_slopes(frame)

            # print slopes
            print(f"Left slope:  {sl_left:.3f}" if sl_left is not None else "Left slope:  —")
            print(f"Right slope: {sl_right:.3f}" if sl_right is not None else "Right slope: —")

            # c) overlay
            disp = frame.copy()
            # draw white lines in cyan
            if lines is not None:
                for x1,y1,x2,y2 in lines.reshape(-1,4):
                    cv2.line(disp, (x1,y1), (x2,y2), (255,255,0), 2)
            # draw midpoints polyline in red
            if len(pts)>=2:
                arr = np.array(pts, np.int32).reshape(-1,1,2)
                cv2.polylines(disp, [arr], False, (0,0,255), 2)

            # show
            cv2.imshow("Edges + Midpoints", disp)
            cv2.imshow("White Mask", wmask)

            if cv2.waitKey(1) == ord('q'):
                break

        device.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
