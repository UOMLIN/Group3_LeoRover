import cv2
import numpy as np
import pyrealsense2 as rs
import supervision as sv
from inference.models.yolo_world.yolo_world import YOLOWorld

# Load YOLO-World model
model = YOLOWorld(model_id="yolo_world/s")

# Set classes for detection
classes = ["a blue cuboid","a red cuboid","a green cuboid"]
model.set_classes(classes)

# Configure Intel RealSense D435i pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)

# Start streaming
pipeline.start(config)

# Define confidence threshold and NMS threshold
confidence_threshold = 0.01
nms_threshold = 0.1

try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Convert the color image to RGB (YOLO-World requires RGB format)
        rgb_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # Run inference on the frame with adjusted confidence level
        results = model.infer(rgb_frame, confidence=confidence_threshold)
        
        # Apply Non-Maximum Suppression
        detections = sv.Detections.from_inference(results).with_nms(threshold=nms_threshold)

        # Annotate the color image with bounding boxes and labels
        BOUNDING_BOX_ANNOTATOR = sv.BoundingBoxAnnotator(thickness=2)
        LABEL_ANNOTATOR = sv.LabelAnnotator(text_thickness=2, text_scale=1, text_color=sv.Color.BLACK)
        labels = [
                    f"{classes[class_id]} {confidence:0.3f}"
                    for class_id, confidence
                    in zip(detections.class_id, detections.confidence)
                ]
        annotated_frame = color_image.copy()
        annotated_frame = BOUNDING_BOX_ANNOTATOR.annotate(annotated_frame, detections)
        annotated_frame = LABEL_ANNOTATOR.annotate(annotated_frame, detections, labels=labels)
        # Display information for detected objects after NMS
        for detection in detections:
            label = detection[0]  # The label is now the first element in the tuple
            confidence = detection[1]  # The confidence is now the second element in the tuple
            if confidence is not None:
                # Get pixel coordinates
                x_pixel = int(detection.center.x)
                y_pixel = int(detection.center.y)

                # Get depth data at the pixel coordinates
                depth_value = depth_frame.get_distance(x_pixel, y_pixel)

                # Convert pixel coordinates to world coordinates
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x_pixel, y_pixel], depth_value)

                # Convert XYZ coordinates to a string
                xyz_str = f"({depth_point[0]:.2f}, {depth_point[1]:.2f}, {depth_point[2]:.2f})"
                
                # Display information on the frame
                cv2.putText(annotated_frame, f"Class: {label}", (x_pixel, y_pixel), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(annotated_frame, f"Confidence: {confidence:.3f}", (x_pixel, y_pixel + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(annotated_frame, f"Depth: {depth_value:.2f} meters", (x_pixel, y_pixel + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(annotated_frame, f"XYZ Coordinates: {xyz_str}", (x_pixel, y_pixel + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Display the annotated frame
        cv2.imshow('YOLO-World Object Detection', annotated_frame)

        # Break the loop when 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()

