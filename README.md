
# LEO ROVER GROUP-3
<p align="center">
  <img src="https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/Final%20Leo.jpg?raw=true?raw=true" alt="Leo Rover" width="800">
  <br>
  <em>Leo Rover</em>
</p>

<p align="center">
  <img src="https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/Group_3_new.jpeg?raw=true" alt="Our Team" width="800">
  <br>
  <em>Our Team</em>
</p>

## Check out the Glimplse!
https://github.com/UOMLIN/Group3_LeoRover/assets/112091438/49de4f61-89bc-4516-8ed4-0ec84178d417


## Table of Contents
- [Who are we?](#who-are-we)
- [Bridging Theory and Real-World Robotics](#robot-implementation)
- [Autonomous Navigation and SLAM](#autonomous-navigation-and-slam)
- [Object Detection](#object-detection)
- [Component Interaction](#component-interaction)
- [Electrical Design](#electrical-design)
  - [Power Connection diagram](#power-connection-diagram)
  - [Components](#components)
  - [Power Connections Explanations](#power-connections-explanations)
  - [Power Analysis](#power-analysis)
- [Software Design](#software-design)
- [Mechanical Design](#mechanical-design)

## Who are we? <a name="who-are-we"></a>
<p align="center">
  <img src="https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/Team3.jpg?raw=true" alt="Team" width="600">
  <br>
  <em>(Left to Right) Jingchuan Lin, Ionut Ionita, Thomas Madeley, Adarsh Karan Kesavadas Prasanth
</em>
</p>

We are a team of four passionate robotics engineers from the University of Manchester's MSc Robotics cohort (2023/2024 batch):
- Jingchuan Lin
- Ionut Ionita
- Thomas Madeley
- Adarsh Karan Kesavadas Prasanth

### Bridging Theory and Real-World Robotics <a name="robot-implementation"></a>

Our year-long capstone project, the LEO Rover Design Project, provided a valuable opportunity to bridge the gap between theoretical knowledge and the practical challenges of real-world robotics. We successfully designed, developed, and tested a functional LEO rover equipped with the following capabilities:

- **Autonomous Exploration and Navigation:** Utilizing LiDAR technology, the LEO rover can autonomously explore and navigate unknown environments. This feat involves obstacle detection, path planning algorithms, and real-time decision-making to ensure safe and efficient exploration.
- **Intelligent Object Detection:** Employing an Intel RealSense depth camera, the LEO rover can effectively detect target objects within its surroundings. This object detection is critical for identifying and locating the desired items for manipulation.
- **Precise Object Retrieval:** Equipped with a 5-DoF manipulator arm, the LEO rover can successfully retrieve the identified target object. This complex task requires accurate object recognition, dexterous arm control, and sophisticated manipulation techniques.

Each of the functions is explained in the following sections:

### Autonomous Navigation and SLAM <a name="autonomous-navigation-and-slam"></a>
A navigation algorithm has been developed for the Leo Rover to autonomously explore unknown environments and return to its initial location. The navigation utilizes the Nav2 stack in ROS2, while mapping is conducted using the SLAM_toolbox. The rover localizes itself using data from Lidar, odometry, and IMU sensors in the simulated Gazebo world. Exploration is facilitated through Frontier Based Exploration techniques. 
<p align="center">
  <img src="https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/navigation&slam/LeoRover%20navigating%20autonomously%20in%20an%20unknown%20environment.png?raw=true" alt="Navigation" width="600">
  <br>
  <em>LeoRover navigating autonomously in an unknown environment</em>
</p>
The entire setup is visualized using the Rviz tool and must now be implemented on the real rover for testing in real-world environments. Additionally, parameters will require fine-tuning based on tests conducted in the actual environment.
<p align="center">
  <img src="https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/navigation&slam/RViz%20Navigation.png?raw=true" alt="Visualization" width="600">
  <br>
  <em>RViz Visualization</em>
</p>

### Object Detection<a name="object-detection"></a>

For object detection, an Intel RealSense RGB-D camera D435i was employed in conjunction with a YOLOv7 model. The model, trained on a custom dataset of colored blocks, successfully identifies objects and outputs their positions (x, y, and z coordinates) along with their orientation (flat or upright) relative to the surface.

### Component Interaction <a name="component-interaction"></a>

- The **Battery** supplies power to the **Power Distribution System**. This Power Distribution System manages the power and distributes it to the **Control system** and the **robot arm**.

- **Intel NUC 12 Pro** serves as the main control unit communicating with **Raspberry Pi 4B** and **LeoCore** via ROS2, processing data from the **RPLIDAR** and **RealSense camera** and controlling the **PincherX 150 robot arm**.

- The **RPLIDAR**, mounted on the Leo Rover, is used to build the map and occupancy grid using **SLAM** to perform Autonomous Navigation.

- The **WiFi Adapter** provides network connectivity through the **WiFi Antenna**.

- The **Raspberry Pi 4B** processes visual data from the **RGB camera** and handles wireless communication.

- The **LeoCore** directly controls the **Wheels**, each powered by a Buehler DC motor with an encoder.

- The **PincherX 150 robot arm** receives the object coordinate information from the depth camera and grabs the object. After grabbing, it puts it into the rear basket and returns to the resting position of the manipulator.

![System_block_diagram](https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/System_block_diagram.png)

## Electrical Design <a name="electrical-design"></a>

### Power Connection diagram <a name="power-connection-diagram"></a>

![Power Connections](https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/LeoRover_powerConnections.png)

### Components <a name="components"></a>

**Wheels**
- **Motors:** 4 x in-hub DC motor with 73.2:1 planetary gearbox and 12 CPR encoder
- **Wheel diameter:** 130 mm
- **Tire material:** rubber with foam insert (non-pneumatic)

**Battery**
- **Voltage:** 11.1 V DC
- **Capacity:** 5800 mAh
- **Type:** Li-Ion with internal PCM (short-circuit, overcurrent, and overdrain safety features)
- **Max. current:** 8A (total for the whole Rover)

**Network**
- **Primary modem:** WiFi 2.4 GHz access point with an external antenna.
- **Secondary modem:** WiFi 2.4 GHz + 5 GHz on internal RPi antennas for connectivity

**Sensors**
- **Lidar:** RPLiDAR A2M12 (360° 2D Scanner)
- **Camera:** Intel RealSense Camera D435i
- **Depth camera:** RGB Camera 5 MPx (Fisheye lens)
- **IMU:** Grove - IMU 9DOF v2.0

**Control**
- **Main computer:** Intel NUC 12 Pro
- **Second computer:** RaspberryPi 4B
- **Electronics board:** LeoCore as a real-time microcontroller with STM32F4

**Manipulator:**
- **PincherX 150 Robot Arm**

### Power Connections Explanations <a name="power-connections-explanations"></a>

#### Controller 
The power management system of the rover is coordinated by the LeoCore Controller, serving a central role in both power distribution and motor control. This controller seamlessly interfaces with the Raspberry Pi 4B and the Intel NUC 12 Pro through UART and RJ 45 Ethernet connections, respectively.

#### Wireless Communication
- Wireless connectivity is managed through the Raspberry Pi 4B, functioning as a central hub for communication, for the Alfa AWUS036ACS AC600 Wireless Adapter integrated via the Universal Serial Bus (USB) interface.
- The Intel NUC 12 Pro establishes connections with sensors—RPLiDAR, Intel RealSense D435i, and PincherX 150 Robot Arm—all operating through USB interfaces.

#### Motor Control
- Utilizing Pulse Width Modulation (PWM) techniques, the LeoCore Controller ensures precise control over four powerful 12V Buehler Brushless DC motors, each with a maximum torque of 1 Nm.
- The LeoCore Controller also interfaces with the Power Distribution System Box via its dedicated PWR port, ensuring an optimal power supply to all connected components.

#### Visual Perception
- A 5MPx RGB Camera, equipped with a fisheye lens and operating at 5V, interfaces with the Raspberry Pi 4B via the MIPI Camera Serial Interface (CSI)-2 interface.
- Additionally, the RPLiDAR A2M12 2D Laser Range Scanner, connected to the Intel NUC through a UART-to-USB converter and operating at 5V, captures data for autonomous navigation, employing SLAM technology.
- Further, the Intel RealSense D435i Depth Camera, which operates at 3.3V, is instrumental in object identification.

#### Manipulator
- The PincherX 150 Robot Arm is equipped with a U2D2 controller, which is a USB to Transistor-Transistor logic (TTL) converter, enabling precise control of DYNAMIXEL servos.
- Simultaneously, the Power Hub Board, operating at 12V, facilitates efficient power distribution from the Power Distribution System to individual DYNAMIXEL servos.

#### Power
- At the core of the Leo Rover's energy source is an 11.1V, 5800mAh Li-Ion battery, capable of delivering a maximum current of 8A.
- Charging is facilitated through the Power Distribution System, utilizing a 12V AC to DC adapter.
- An LED indicator on the battery unit provides a visual cue, indicating the operational status of the rover.

### Power Analysis <a name="power-analysis"></a>

| Components                                    | Voltage(max) (V) | Current(max) (A) | Power(max) (W) |
|-----------------------------------------------|------------------|-------------------|----------------|
| LeoCore Controller                            | 12               | 3                 | 36             |
| RaspberryPI 4B                                | 5                | 3                 | 15             |
| Wheels Buehler Motors 1.61.077.414           | 12               | 0.86 (stall current) | 41.3 approx   |
| Wheel Encoders - Pololu Romi magnetic encoders | 12               | 0.006             | 0.288 approx  |
| 2.4 GHz Antenna module                        | 3.3              | 0.115 (in transmit mode) | 0.38 approx   |
| Intel NUC 12 Pro                              | 12               | -                 | 100 (peak under load) |
| RPLiDAR A2M12                                | 5                | 0.6               | 3              |
| Intel RealSense D435i                        | 5                | 0.7               | 3.5            |
| 5MP RGB Camera                                | 3.3              | 0.25              | 0.83           |
| PincherX 150 Robot Arm - DYNAMIXEL U2D2 power hub | 


### Software Design <a name="software-design"></a>


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

### Mechanical Design <a name="mechanical-design"></a>

Mechanically there are two main areas of the project: the equipment mounting structure, and the payload container. The former shall support all the sensors, computing equipment, the manipulator and any other components necessary for the operation of the robot, while the container’s purpose is to store the payload items collected by the arm and deposit them on the ground once the desired rover pose is reached.

<p align="center">
  <img src="https://github.com/UOMLIN/Group3_LeoRover/blob/main/Pictures/Mech%20Des1.png?raw=true" alt="Mechanical Design" width="800">
  <br>
  <em>Mechanical Design
</em>
</p>

## Requirements Video

https://github.com/UOMLIN/Group3_LeoRover/assets/112091438/dee035ed-5fe2-494e-ad97-151e56923ac9

