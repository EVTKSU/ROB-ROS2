#!/usr/bin/env python3
import cv2
import depthai as dai
import numpy as np
from datetime import datetime
from pathlib import Path

def main():
    # 1) Prepare recordings folder
    rec_dir = Path.cwd() / "recordings"
    rec_dir.mkdir(parents=True, exist_ok=True)

    # 2) Generate timestamped filename
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    mp4_path = rec_dir / f"thermal_{now}.mp4"

    # 3) Recording settings
    FPS    = 20
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = None
    font   = cv2.FONT_HERSHEY_SIMPLEX

    # 4) Build DepthAI thermal pipeline
    with dai.Pipeline() as pipeline:
        thermal      = pipeline.create(dai.node.Thermal)
        qTemperature = thermal.temperature.createOutputQueue()
        qColor       = thermal.color.createOutputQueue()
        pipeline.start()

        # 5) Windows for preview
        MAGMA_WINDOW = "Colorized Temperature"
        RAW_WINDOW   = "Thermal image"
        cv2.namedWindow(MAGMA_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(RAW_WINDOW,   cv2.WINDOW_NORMAL)
        initialSetup = False

        print(f"Waiting for thermal data to start recording to {mp4_path}")
        try:
            while True:
                # Grab frames
                inTemp  = qTemperature.get()
                inColor = qColor.get()

                # Convert to 2D float32 array
                thermalData = (
                    inTemp.getData()
                          .view(dtype=np.float16)
                          .reshape((inTemp.getHeight(), inTemp.getWidth()))
                          .astype(np.float32)
                )

                # Normalize and apply colormap
                norm = cv2.normalize(thermalData, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                colormapped = cv2.applyColorMap(norm, cv2.COLORMAP_MAGMA)

                # Compute min/max temperature in °C
                temp_min = thermalData.min()
                temp_max = thermalData.max()

                # Overlay timestamp and temp stats
                ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(colormapped, ts, (10, 25), font, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(colormapped, f"Min: {temp_min:.1f}°C", (10, 50), font, 0.6, (255,255,255), 1, cv2.LINE_AA)
                cv2.putText(colormapped, f"Max: {temp_max:.1f}°C", (10, 75), font, 0.6, (255,255,255), 1, cv2.LINE_AA)

                # One-time window sizing & positioning
                if not initialSetup:
                    h, w = colormapped.shape[:2]
                    cv2.resizeWindow(MAGMA_WINDOW, w*4,  h*4)
                    cv2.moveWindow(RAW_WINDOW,   w*4,  0)
                    cv2.resizeWindow(RAW_WINDOW, w*4,  h*4)
                    initialSetup = True

                # Initialize writer once we know frame size
                if writer is None:
                    writer = cv2.VideoWriter(
                        str(mp4_path), fourcc, FPS,
                        (colormapped.shape[1], colormapped.shape[0])
                    )
                    print(f"Recording to {mp4_path} @ {FPS} FPS, "
                          f"{colormapped.shape[1]}x{colormapped.shape[0]}")

                # Write only the colormapped frame
                writer.write(colormapped)

                # Display
                cv2.imshow(MAGMA_WINDOW, colormapped)
                cv2.imshow(RAW_WINDOW,   inColor.getCvFrame())

                if cv2.waitKey(1) == ord('q'):
                    break
        finally:
            if writer:
                writer.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
