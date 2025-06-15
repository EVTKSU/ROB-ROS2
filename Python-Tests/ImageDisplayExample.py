#!/usr/bin/env python3
import os
# 1) Force OpenCV’s Qt to pick the XCB backend (avoid missing 'wayland' plugin)
os.environ["QT_QPA_PLATFORM"] = "xcb"

import cv2
import time
import depthai as dai

class ImageFolderSource(dai.node.ThreadedHostNode):
    """
    Host-side node that streams images from a local folder into the pipeline.
    """
    def __init__(self, folder="Test_Images"):
        super().__init__()
        self.folder = folder
        self.output = self.createOutput()  # v3 API for host-node outputs

    def run(self):
        global p  # refer to the pipeline so we can stop it
        # Gather images
        files = sorted([
            f for f in os.listdir(self.folder)
            if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp'))
        ])
        if not files:
            print(f"[Error] No images found in '{self.folder}'")
            p.stop()  # gracefully terminate pipeline
            return

        # Stream each image
        for fn in files:
            if not self.isRunning():
                break
            path = os.path.join(self.folder, fn)
            frame = cv2.imread(path)
            if frame is None:
                print(f"[Warning] Could not load '{path}'")
                continue

            msg = dai.ImgFrame()
            msg.setData(frame)
            msg.setWidth(frame.shape[1])
            msg.setHeight(frame.shape[0])
            msg.setType(dai.ImgFrame.Type.BGR888i)
            self.output.send(msg)

            time.sleep(5.5)  # adjust display rate

        # After all images, stop the pipeline
        p.stop()

def main():
    # Use createImplicitDevice=False so we can manually call p.start()/p.stop()
    with dai.Pipeline(createImplicitDevice=False) as p:
        source = p.create(ImageFolderSource, folder="Test_Images")
        queue = source.output.createOutputQueue()

        p.start()

        # Display loop
        while p.isRunning():
            inMsg = queue.tryGet()
            if inMsg:
                cv_frame = inMsg.getCvFrame()
                cv2.imshow("Test_Images Stream", cv_frame)
                if cv2.waitKey(1) == ord('q'):
                    p.stop()
                    break
            else:
                # no frame available right now
                time.sleep(0.01)

        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
