Function: Based on three sensors, output wheel speed, and control the robot's on/off switch. 

If the depth camera detects an object and the distance is within 5 cm, return true and stop the movement. 

After stopping the movement, wait for the Manipulator to finish grabbing the item (return True) before starting to move. 

Input: radar (float), depth camera (float, Boolean), Leo camera (float) 

Output: Wheel speed (float) 
