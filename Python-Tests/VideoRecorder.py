#!/usr/bin/env python3
import os
import cv2
import depthai as dai
from datetime import datetime
from pathlib import Path

def main():
    # 1) Prepare recordings folder (no nested timestamp dir)
    rec_dir = Path.cwd() / "recordings"
    rec_dir.mkdir(parents=True, exist_ok=True)

    # 2) Generate timestamped filename
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    mp4_path = rec_dir / f"video_{now}.mp4"

    # 3) Recording settings
    FPS        = 20
    RESOLUTION = (640, 480)  # width, height

    # 4) Build DepthAI v3 pipeline
    with dai.Pipeline() as pipeline:
        cam = pipeline.create(dai.node.Camera).build()
        preview_q = cam.requestOutput(
            size=RESOLUTION,
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.LETTERBOX,
            fps=FPS
        ).createOutputQueue()

        pipeline.start()

        # 5) OpenCV VideoWriter for MP4
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(str(mp4_path), fourcc, FPS, RESOLUTION)

        print(f"Recording to {mp4_path} @ {FPS} FPS, {RESOLUTION[0]}x{RESOLUTION[1]}")

        try:
            while True:
                frame = preview_q.get().getCvFrame()
                writer.write(frame)
                cv2.imshow("Recording Preview", frame)
                if cv2.waitKey(1) == ord('q'):
                    break
        finally:
            writer.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
