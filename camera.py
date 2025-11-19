from inference import InferencePipeline
import cv2
import cscore as cs
import numpy as np
import os
from dotenv import load_dotenv

camera_server = cs.CameraServer.getInstance()
camera_server.enableLogging()
output_stream = camera_server.putVideo("DetectionFeed", 640, 480)
load_dotenv()

def my_sink(result, video_frame):
    print(result)

    if result.get("output_image"):
        image = result["output_image"].numpy_image
        
        if len(image.shape) == 3 and image.shape[2] == 3:
            frame_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        else:
            frame_bgr = image

        output_stream.putFrame(frame_bgr)
        
    # cv2.imshow("Workflow Image", frame_bgr)
    # cv2.waitKey(1)

pipeline = InferencePipeline.init_with_workflow(
    api_key= os.getenv("ROBOFLOW_API_KEY"),
    workspace_name="frc-offseason",
    workflow_id="small-object-detection-sahi",
    video_reference="http://localhost:1181/?action=stream",
    max_fps=30,
    on_prediction=my_sink
)

pipeline.start()
pipeline.join()