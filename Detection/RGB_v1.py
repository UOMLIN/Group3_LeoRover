# -*- coding: utf-8 -*-
"""
Created on Fri Feb 16 09:42:14 2024

@author: Johnny
"""

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

# Depth scale
depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # Color segmentation to detect green objects
        hsv_frame = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        # Green color range
       # Define range of darker green color in HSV
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([90, 255, 255])
    
        # Threshold the HSV image to get only darker green colors
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)

    
        # Color segmentation to detect red objects
        # Red color range
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        red_mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
           
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        red_mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
           
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
   
        # Color segmentation to detect blue objects
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        # Draw bounding boxes around detected green objects
        object_count = 0
        
        # Find contours in the green mask
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Filter contours based on area
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)

            if area > 20:
                object_count += 1

                # Calculate the centroid of the object
                cX = x + w // 2
                cY = y + h // 2

                # Get the depth value at the centroid
                depth_value = depth_frame.get_distance(cX, cY)

                # Check if the depth value is valid
                if depth_value > 0:
                    # Convert pixel dimensions to real-world dimensions
                    length_cm = (w * depth_value * depth_scale)*100 +1
                    width_cm = (h * depth_value * depth_scale) *100 +1

                    # Draw bounding boxes
                    cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                    # Display the dimensions under the bounding box
                    cv2.putText(color_image, f"Length: {length_cm:.2f} cm", (x, y+h+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(color_image, f"Width: {width_cm:.2f} cm", (x, y+h+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Print XYZ coordinates if needed
                    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], depth_value)
                    cv2.putText(color_image, "Object {}: Green XYZ: {:.2f}, {:.2f}, {:.2f}".format(object_count, depth_point[0], depth_point[1], depth_point[2]), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
                    
                    # Display the dimensions under the bounding box
                    cv2.putText(color_image, f"Length: {length_cm:.2f} cm", (x, y+h+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(color_image, f"Width: {width_cm:.2f} cm", (x, y+h+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                
            # Find contours in the red mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Filter contours based on area
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
         
            if area > 20:
                object_count += 1
         
                # Calculate the centroid of the object
                cX = x + w // 2
                cY = y + h // 2
         
                # Get the depth value at the centroid
                depth_value = depth_frame.get_distance(cX, cY)
         
                # Check if the depth value is valid
                if depth_value > 0:
                    # Convert pixel dimensions to real-world dimensions
                    length_cm = (w * depth_value * depth_scale)*100 +1
                    width_cm = (h * depth_value * depth_scale) *100 +1
         
                    # Draw bounding boxes
                    cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
         
                    # Display the dimensions under the bounding box
                    cv2.putText(color_image, f"Length: {length_cm:.2f} cm", (x, y+h+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(color_image, f"Width: {width_cm:.2f} cm", (x, y+h+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
         
                    # Print XYZ coordinates if needed
                    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], depth_value)
                    cv2.putText(color_image, "Object {}: Red XYZ: {:.2f}, {:.2f}, {:.2f}".format(object_count, depth_point[0], depth_point[1], depth_point[2]), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
                    
                    # Display the dimensions under the bounding box
                    cv2.putText(color_image, f"Length: {length_cm:.2f} cm", (x, y+h+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    cv2.putText(color_image, f"Width: {width_cm:.2f} cm", (x, y+h+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        
       # Draw bounding boxes around detected blue objects
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            # Filter contours based on area
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
         
            if area > 20:
                object_count += 1
         
                # Calculate the centroid of the object
                cX = x + w // 2
                cY = y + h // 2
         
                # Get the depth value at the centroid
                depth_value = depth_frame.get_distance(cX, cY)
         
                # Check if the depth value is valid
                if depth_value > 0:
                    # Convert pixel dimensions to real-world dimensions
                    length_cm = (w * depth_value * depth_scale)*100 +1
                    width_cm = (h * depth_value * depth_scale) *100 +1
         
                    # Draw bounding boxes
                    cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
         
                    # Display the dimensions under the bounding box
                    cv2.putText(color_image, f"Length: {length_cm:.2f} cm", (x, y+h+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(color_image, f"Width: {width_cm:.2f} cm", (x, y+h+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
         
                    # Print XYZ coordinates if needed
                    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [cX, cY], depth_value)
                    cv2.putText(color_image, "Object {}: Blue XYZ: {:.2f}, {:.2f}, {:.2f}".format(object_count, depth_point[0], depth_point[1], depth_point[2]), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
                    
                    # Display the dimensions under the bounding box
                    cv2.putText(color_image, f"Length: {length_cm:.2f} cm", (x, y+h+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    cv2.putText(color_image, f"Width: {width_cm:.2f} cm", (x, y+h+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    # Show images
        cv2.imshow('Color Image', color_image)
        depthFrameColor = cv2.normalize(depth_image, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_JET)
        cv2.imshow('Depth Image', depthFrameColor)

        if cv2.waitKey(1) == ord('q'):
            break

finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
