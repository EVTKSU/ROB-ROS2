#!/usr/bin/env python3
import os
import cv2
import depthai as dai
import numpy as np
from depthai_nodes.node import ParsingNeuralNetwork

def main():
    # 1) Create pipeline using v3 builder API
    with dai.Pipeline() as pipeline:
        # 2) Set up unified Camera node (CAM_A by default)
        cam = pipeline.create(dai.node.Camera).build()  

        # 3) Request a BGR-formatted preview at the model’s input size (800×288)
        video_q = cam.requestOutput((800, 288)).createOutputQueue()

        # 4) Describe the Ultra-Fast Lane Detection model
        model_description = dai.NNModelDescription(
            "luxonis/ultra-fast-lane-detection:culane-800x288"
        )  # :contentReference[oaicite:0]{index=0}

        # 5) Create a ParsingNeuralNetwork to run inference + parsing on-device
        nn = pipeline.create(ParsingNeuralNetwork).build(cam, model_description)

        # 6) Grab parsed lane clusters from the network output
        lanes_q = nn.out.createOutputQueue()

        # 7) Start everything
        pipeline.start()

        while pipeline.isRunning():
            # 8) Get new video frame
            in_frame = video_q.get()
            frame = in_frame.getCvFrame()

            # 9) Fetch latest lane detections (Clusters message)
            lanes = lanes_q.tryGet()
            if lanes:
                # Each `cluster` has a list of normalized points for one lane
                for cluster in lanes.clusters:
                    pts = cluster.points
                    # Draw a polyline through all points in this lane
                    if len(pts) >= 2:
                        coords = [(
                            int(p.x * frame.shape[1]), 
                            int(p.y * frame.shape[0])
                        ) for p in pts]
                        cv2.polylines(frame, [np.array(coords)], False, (0,255,0), 2)

            # 10) Display
            cv2.imshow("Ultra-Fast Lane Detection", frame)
            if cv2.waitKey(1) == ord('q'):
                break

        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
