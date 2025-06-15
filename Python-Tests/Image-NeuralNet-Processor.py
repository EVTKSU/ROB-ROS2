#!/usr/bin/env python3
import os
# Force OpenCV’s Qt backend to XCB (avoid missing Wayland plugin)
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import tempfile
from pathlib import Path
import time
import depthai as dai
import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork

# ─── Configuration ───────────────────────────────────────────────
SOURCE_FOLDER    = "Test_Images"
FRAME_DELAY_SEC  = 0.05               # seconds each image should display
TEMP_VIDEO_NAME  = "folder_replay.mp4"

def make_video_from_folder(folder: str, output_file: str, delay_s: float):
    files = sorted([
        os.path.join(folder, f)
        for f in os.listdir(folder)
        if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp'))
    ])
    if not files:
        raise RuntimeError(f"No images found in '{folder}'")
    first = cv2.imread(files[0])
    if first is None:
        raise RuntimeError(f"Cannot read '{files[0]}'")
    h, w = first.shape[:2]
    fps = 1.0 / delay_s
    writer = cv2.VideoWriter(output_file,
                             cv2.VideoWriter_fourcc(*'mp4v'),
                             fps, (w, h))
    for path in files:
        img = cv2.imread(path)
        if img is None:
            print(f"[Warning] skipping unreadable '{path}'")
            continue
        if (img.shape[1], img.shape[0]) != (w, h):
            img = cv2.resize(img, (w, h))
        writer.write(img)
    writer.release()
    print(f"[Done] Created video {output_file} @ {fps:.2f} FPS")

def main():
    # 1) Build the MP4
    temp_mp4 = os.path.join(tempfile.gettempdir(), TEMP_VIDEO_NAME)
    make_video_from_folder(SOURCE_FOLDER, temp_mp4, FRAME_DELAY_SEC)

    # 2) Build the DepthAI pipeline
    with dai.Pipeline() as pipeline:
        # Replay folder MP4 on-device
        replay = pipeline.create(dai.node.ReplayVideo)
        replay.setReplayVideoFile(Path(temp_mp4))
        replay.setLoop(True)
        replay.setOutFrameType(dai.ImgFrame.Type.BGR888p)

        # Ultra-fast lane detection descriptor
        model_desc = dai.NNModelDescription(
            "luxonis/ultra-fast-lane-detection:culane-800x288"
        )

        # ParsingNeuralNetwork node
        pnn = pipeline.create(ParsingNeuralNetwork)
        pnn.build(replay, model_desc)  # (ReplayVideo, NNModelDescription) overload :contentReference[oaicite:0]{index=0}
        lanes_q  = pnn.out.createOutputQueue()
        frames_q = replay.out.createOutputQueue()

        # 3) Start the device
        device = pipeline.start()

        # 4) Display loop with enforced FRAME_DELAY_SEC per image
        last_time = None
        while pipeline.isRunning():
            start = time.monotonic()

            in_frame = frames_q.get()
            frame    = in_frame.getCvFrame()

            # Draw lanes
            lanes = lanes_q.tryGet()
            if lanes:
                for cluster in lanes.clusters:
                    pts = cluster.points
                    if len(pts) >= 2:
                        coords = [
                            (int(p.x * frame.shape[1]), int(p.y * frame.shape[0]))
                            for p in pts
                        ]
                        cv2.polylines(frame, [np.array(coords)], False, (0,255,0), 2)

            cv2.imshow("Folder + Lane Detection", frame)
            if cv2.waitKey(1) == ord('q'):
                break

            # Enforce the remainder of FRAME_DELAY_SEC
            elapsed = time.monotonic() - start
            to_sleep = FRAME_DELAY_SEC - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

        device.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
