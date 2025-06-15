#!/usr/bin/env python3
import os
import cv2
import numpy as np
import tempfile
import time
from pathlib import Path
import depthai as dai

# ─── Config ─────────────────────────────────────────────────────────
SOURCE_FOLDER   = "Test_Images"
FRAME_DELAY_SEC = .25  # seconds per frame
TEMP_VIDEO_NAME = "folder_replay.mp4"
Y_MIN_RATIO     = 0.4  # ignore top 40% (sky) rows

# PID gains
KP = 0.6
KI = 0.02
KD = 0.1
INTEGRAL_LIMIT = 1000

class PID:
    def __init__(self, kp, ki, kd, windup=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.windup = windup
        self.integral = 0.0
        self.prev_error = 0.0
    def update(self, err, dt):
        p = self.kp * err
        self.integral += err * dt
        if self.windup:
            self.integral = max(-self.windup, min(self.integral, self.windup))
        i = self.ki * self.integral
        d = self.kd * ((err - self.prev_error) / dt if dt>0 else 0.0)
        self.prev_error = err
        return p + i + d

def make_video_from_folder(folder, out_file, delay):
    imgs = sorted(f for f in os.listdir(folder)
                  if f.lower().endswith(('.png','.jpg','.jpeg','.bmp')))
    if not imgs:
        raise RuntimeError(f"No images in '{folder}'")
    first = cv2.imread(os.path.join(folder, imgs[0]))
    h, w = first.shape[:2]
    fps = 1.0 / delay
    vw = cv2.VideoWriter(out_file,
                         cv2.VideoWriter_fourcc(*'mp4v'),
                         fps, (w, h))
    for fn in imgs:
        im = cv2.imread(os.path.join(folder, fn))
        if im is None: continue
        if (im.shape[1], im.shape[0]) != (w, h):
            im = cv2.resize(im, (w, h))
        vw.write(im)
    vw.release()

def segment_asphalt(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array([0,   0,  80]),
                       np.array([180, 60, 255]))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask

def compute_midpoints_via_distance(mask, y_min):
    # 1) Distance transform
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)  # :contentReference[oaicite:5]{index=5}
    h, _ = mask.shape
    pts = []
    for y in range(y_min, h):
        x = int(np.argmax(dist[y]))  # :contentReference[oaicite:6]{index=6}
        pts.append((x, y))
    return pts

def main():
    # 1) Build and replay video
    tmp = os.path.join(tempfile.gettempdir(), TEMP_VIDEO_NAME)
    make_video_from_folder(SOURCE_FOLDER, tmp, FRAME_DELAY_SEC)

    pid = PID(KP, KI, KD, INTEGRAL_LIMIT)
    prev_t = time.monotonic()
    smooth_x = None

    # DepthAI pipeline
    with dai.Pipeline() as pipeline:
        rv = pipeline.create(dai.node.ReplayVideo)
        rv.setReplayVideoFile(Path(tmp))
        rv.setLoop(True)
        rv.setOutFrameType(dai.ImgFrame.Type.BGR888p)
        q = rv.out.createOutputQueue()
        device = pipeline.start()

        y_min = None
        while pipeline.isRunning():
            t0 = time.monotonic()
            msg   = q.get()
            frame = msg.getCvFrame()
            h, w  = frame.shape[:2]
            if y_min is None:
                y_min = int(Y_MIN_RATIO * h)

            # 2) Segment mask & compute midpoints
            mask = segment_asphalt(frame)
            pts  = compute_midpoints_via_distance(mask, y_min)

            # 3) Smooth curve via 2nd-order polyfit
            if len(pts) >= 3:
                ys, xs = zip(*pts)
                coeff = np.polyfit(ys, xs, 2)
                smooth_pts = [(int(np.polyval(coeff, y)), y) for y in ys]
                x_bot = int(np.polyval(coeff, h-1))
            elif pts:
                smooth_pts = pts
                x_bot = pts[-1][0]
            else:
                smooth_pts = []
                x_bot = w//2

            # 4) PID on bottom error
            now = time.monotonic()
            dt  = max(now - prev_t, 1e-3)
            prev_t = now
            error   = x_bot - (w//2)
            control = pid.update(error, dt)
            smooth_x = int((w//2) + control) if smooth_x is not None else x_bot

            # 5) Overlay: raw midpoints (red), poly (blue), PID centerline (green)
            disp = frame.copy()
            for x,y in pts:
                cv2.circle(disp, (x,y), 2, (0,0,255), -1)
            if len(smooth_pts) >= 2:
                arr = np.array(smooth_pts, np.int32).reshape(-1,1,2)
                cv2.polylines(disp, [arr], False, (255,0,0), 2)
            cv2.line(disp, (smooth_x,0), (smooth_x,h-1), (0,255,0), 2)

            cv2.imshow("Distance-Transform Midpoints + PID", disp)
            if cv2.waitKey(1) == ord('q'):
                break

            # 6) Enforce frame delay
            elapsed = time.monotonic() - t0
            if elapsed < FRAME_DELAY_SEC:
                time.sleep(FRAME_DELAY_SEC - elapsed)

        device.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
