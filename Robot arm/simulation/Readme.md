Install rosdep package using following command:

    sudo apt install python3-rosdep2
    
    rosdep update
Follow the step on offical documents to the step ROS 2 Quickstart Guide:

https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html

Start simulation:

    source install/setup.bash
    
    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150

RVIZ simulation

1. The whole package could be 'colcon build' when connecting to the manipulator
   
2. Run the control file
   
3.Under folder interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/demos/python_ros2_api can change the xyz pose.  

Grasping:

1. Grasping raw code inside
2.     interbotix_ros_arms/interbotix_sdk/src/interbotix_sdk/robot_manipulation.py
3.     function:open_gripper
