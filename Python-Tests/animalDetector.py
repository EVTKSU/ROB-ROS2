#!/usr/bin/env python3
import os
import cv2
import depthai as dai

# 3 classes: animal, person, vehicle
labelMap = ["animal", "person", "vehicle"]  # model has 3 output classes :contentReference[oaicite:8]{index=8}

def main():
    # Build pipeline
    with dai.Pipeline() as pipeline:
        # Unified Camera node (CAM_A)
        cam = pipeline.create(dai.node.Camera).build()  # v3 builder API :contentReference[oaicite:9]{index=9}

        # Preview stream at model input size
        preview_q = cam.requestOutput(
            size=(512, 288),
            type=dai.ImgFrame.Type.BGR888p,
            fps=15
        ).createOutputQueue()  # matches [1,3,288,512] input :contentReference[oaicite:10]{index=10}

        # Detection network with Wildlife MegaDetector blob
        model_desc = dai.NNModelDescription("luxonis/wildlife-megadetector:mdv6-yolov10-c")
        detNet = pipeline.create(dai.node.DetectionNetwork).build(cam, model_desc)  # on-camera parsing :contentReference[oaicite:11]{index=11}
        detNet.setConfidenceThreshold(0.5)            # filter < 50% confidence :contentReference[oaicite:12]{index=12}
        detNet.input.setBlocking(False)

        # Detection output queue
        det_q = detNet.out.createOutputQueue()        # ImgDetections messages :contentReference[oaicite:13]{index=13}

        # Start pipeline
        pipeline.start()

        # Host loop: fetch frames + detections
        while pipeline.isRunning():
            in_frame   = preview_q.get()               # get preview ImgFrame :contentReference[oaicite:14]{index=14}
            frame      = in_frame.getCvFrame()
            detections = det_q.tryGet()                # non-blocking fetch

            # Draw detections
            if detections:
                for d in detections.detections:
                    x1 = int(d.xmin * frame.shape[1])
                    y1 = int(d.ymin * frame.shape[0])
                    x2 = int(d.xmax * frame.shape[1])
                    y2 = int(d.ymax * frame.shape[0])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                    cls = d.label
                    conf = d.confidence
                    cv2.putText(
                        frame,
                        f"{labelMap[cls]}: {conf:.2f}",
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1
                    )

            # Display
            cv2.imshow("Wildlife MegaDetector", frame)  # OpenCV display :contentReference[oaicite:15]{index=15}
            if cv2.waitKey(1) == ord('q'):
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
