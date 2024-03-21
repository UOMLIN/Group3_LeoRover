1. Data is collected from three sensors, with depth camera data being sent to `ros_gz_bridge` and YOLO Object Detection. Radar data is also sent to `ros_gz_bridge`, and Leo camera data is sent to an external display, allowing users to see the robot's perspective.

2. After receiving information from radar and IMU, `ros_gz_bridge` publishes the data to the Slam toolbox, enabling the robot to map and identify obstacles ahead.

3. Slam toolbox internally completes the map drawing function and publishes `/map` to the Navigation stack. (Proceed to step 5)

4. YOLO Object Detection uses data from the depth camera to determine if a target object is detected. If detected, it publishes a `True` message and the robot's goal position to the Navigation stack.

5. The Navigation stack has two scenarios.
   - In the first scenario, upon receiving a `True` message and the destination from YOLO Object Detection, the robot starts automatic navigation to the target location. After reaching the destination, it publishes a `True` message to Manipulator through the `/goal_reach` topic. Meanwhile, the robot stops moving, waiting for a `True` message from Manipulator indicating successful object grasping.
   - In the second scenario, if the robot has not found a target object, it performs the default operation—moves along the right wall along the Y-axis until YOLO Object Detection publishes a `True` message to the Navigation stack. (Return to step 4)

6. When Manipulator receives a `True` message from the Navigation stack, it subscribes to the `/target_position` information published by YOLO Object Detection to confirm the location of the target object. Using mathematical methods such as rotation matrices, it calculates the relative position of the object and the End Effector, then publishes the updated object position information to the End Effector.

7. After receiving the object position information, the End Effector performs the grasping action. Once it complete, control the arm back to the default position (folded together). It publishes a `True` message through `/end_effector`, indicating the end of the grasping process and enabling the start of the return task.

8. After Manipulator receives the `True` message from the End Effector, it stops receiving information from YOLO Object and informs the Navigation stack Node through `/target_finish`.

9. Upon receiving information from Manipulator, the Navigation stack sets the robot's departure position to the target position and executes the return operation. When the robot returns to the starting point, the entire process is complete.

![Decision Tree](https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/Decision%20Tree.jpg)

![RQT](https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/RQT_Final.jpg)
