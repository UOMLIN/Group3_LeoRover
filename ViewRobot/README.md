This folder contain main launch file of the whole robot
It contains ros_gz_bridge, slam , navigation and twist_mux.
Simulation Publishers:
odometry (diff. drive wheel dead reckoning) - /odom_raw

IMU - /imu_raw

Laser scan - /scan

Depth camera namespace - /depth

Simulation Subscribers:

Command velocity - /robotname/cmd_vel

called “twist_mux”.

Twist Mux Publishers:

 Output command velocity - /robotname/cmd_vel

Twist Mux Subscribers:

 User command velocity - /teleop_cmd_vel

 Autonomous Nav command velocity - /nav_cmd_vel
