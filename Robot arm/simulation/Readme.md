Install rosdep package using following command:

    sudo apt install python3-rosdep2
    
    rosdep update
Follow the step on offical documents to the step ROS 2 Quickstart Guide:

https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html

Start simulation:

    source install/setup.bash
    
    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px150
