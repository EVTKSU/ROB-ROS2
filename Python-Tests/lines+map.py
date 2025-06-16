#!/usr/bin/env python3
import os
# Force OpenCV’s Qt backend to XCB (avoid missing 'wayland' plugin)
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import numpy as np
import tempfile
from pathlib import Path
import depthai as dai

# ─── Configuration ─────────────────────────────────────────
SOURCE_FOLDER   = "Test_Images"
FRAME_DELAY_SEC = 5.5               # seconds per frame
TEMP_VIDEO      = "folder_replay.mp4"
Y_MIN           = 250               # ignore rows above this
# HSV bounds for grass segmentation
GRASS_LOWER_HSV = np.array([25, 40, 40])
GRASS_UPPER_HSV = np.array([95, 255, 255])

def make_video_from_folder(folder, output_file, delay_s):
    """Convert images in `folder` into an MP4 at 1/delay_s FPS."""
    files = sorted(
        f for f in os.listdir(folder)
        if f.lower().endswith(('.png','.jpg','.jpeg','.bmp'))
    )
    if not files:
        raise RuntimeError(f"No images found in '{folder}'")
    first = cv2.imread(os.path.join(folder, files[0]))
    h, w = first.shape[:2]
    fps  = 1.0 / delay_s
    writer = cv2.VideoWriter(
        output_file,
        cv2.VideoWriter_fourcc(*'mp4v'),
        fps,
        (w, h)
    )
    for fn in files:
        img = cv2.imread(os.path.join(folder, fn))
        if img is None:
            continue
        if (img.shape[1], img.shape[0]) != (w, h):
            img = cv2.resize(img, (w, h))
        writer.write(img)
    writer.release()

def segment_grass(frame):
    """Return a binary mask where grass is white."""
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, GRASS_LOWER_HSV, GRASS_UPPER_HSV)
    # Clean small speckles and fill holes
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15,15))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    # Dilate to thicken thin patches
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT,(5,5)), iterations=1)
    return mask

def main():
    # 1) Build the temporary MP4
    tmp_path = os.path.join(tempfile.gettempdir(), TEMP_VIDEO)
    make_video_from_folder(SOURCE_FOLDER, tmp_path, FRAME_DELAY_SEC)

    # 2) Build and start DepthAI ReplayVideo pipeline
    with dai.Pipeline() as pipeline:
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(Path(tmp_path))
        replay.setLoop(True)
        replay.setOutFrameType(dai.ImgFrame.Type.BGR888p)
        frames_q = replay.out.createOutputQueue()
        device   = pipeline.start()

        # 3) Main processing loop
        while pipeline.isRunning():
            in_frame = frames_q.get()
            frame    = in_frame.getCvFrame()
            h, w     = frame.shape[:2]

            # 4) Grass segmentation
            grass_mask = segment_grass(frame)

            # 5) Invert to get road candidates
            road_cand = cv2.bitwise_not(grass_mask)

            # 6) Find largest contour = actual track region
            cnts, _ = cv2.findContours(
                road_cand, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if cnts:
                track_cnt = max(cnts, key=cv2.contourArea)
                track_mask = np.zeros_like(road_cand)
                cv2.drawContours(track_mask, [track_cnt], -1, 255, thickness=cv2.FILLED)
            else:
                track_mask = road_cand

            # 7) Extract left/right edge points per row
            left_pts, right_pts = [], []
            for y in range(Y_MIN, h):
                xs = np.where(track_mask[y] > 0)[0]
                if xs.size:
                    left_pts.append((int(xs[0]), y))
                    right_pts.append((int(xs[-1]), y))

            # 8) Overlay thick red lines on the edges
            disp = frame.copy()
            if len(left_pts) >= 2:
                cv2.polylines(
                    disp,
                    [np.array(left_pts, dtype=np.int32).reshape(-1,1,2)],
                    False,
                    (0,0,255),  # red
                    thickness=8
                )
            if len(right_pts) >= 2:
                cv2.polylines(
                    disp,
                    [np.array(right_pts, dtype=np.int32).reshape(-1,1,2)],
                    False,
                    (0,0,255),
                    thickness=8
                )

            # 9) Display
            cv2.imshow("Asphalt Edges", disp)
            if cv2.waitKey(1) == ord('q'):
                break

        device.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

