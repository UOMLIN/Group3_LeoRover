This folder contains the main launch file of the whole robot
It contains ros_gz_bridge, slam, navigation, and twist_mux.

**Simulation Publishers:**

odometry (diff. drive wheel dead reckoning) - /odom

IMU - /imu

Laser scan - /scan

Depth camera  - /depth

**Simulation Subscribers:**

Command velocity - /robot/cmd_vel

called “twist_mux”.

Twist Mux Publishers:

 Output command velocity - /robot/cmd_vel

Twist Mux Subscribers:

 User command velocity - /teleop_cmd_vel

 Autonomous Nav command velocity - /nav_cmd_vel
