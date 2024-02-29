#Install rosdep package using following command:

    sudo apt install python3-rosdep2
    
    rosdep update
    
Follow the step on offical documents to the step ROS 2 Quickstart Guide:

https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html

#Start simulation:
## First colcon build and launch the rviz

    source install/setup.bash
    
    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150 use_sim:=true

## Next, in another terminal, navigate to the simulation directory and run the command
    python3 bartender.py
Grasping:

1. Grasping raw code inside
2.     interbotix_ros_arms/interbotix_sdk/src/interbotix_sdk/robot_manipulation.py
3.     function:open_gripper
