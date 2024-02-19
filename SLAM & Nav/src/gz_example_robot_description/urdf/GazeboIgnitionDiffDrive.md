# Setting up a Differential Drive Robot in Gazebo

The term "Gazebo" will be used to reference the Fortress version of Ignition Gazebo which ships with ROS2 Humble.  Though technically it should be referred to as "Ignition Gazebo", who really cares?  The newer versions of Gazebo Sim have dropped the ignition branding, and the "old" verison of Gazebo is known as Gazebo Classic.

WARNING!:  Nearly all tutorials you will find online are specifically for Gazebo Classic - we are at the cutting edge using Gazebo Sim/Ignition Gazebo.  Even the official documentation is pretty sparse.

---
## Step 0 - Make a Package

Typically, there would be a _description_ package and a _sim_ or _gazebo_ package.  However, for convenience we will roll it all into one package called "gz_example_robot".

Remember to add directories to your "setup.py" file.  To start with, we will add a _launch_ and _worlds_ directory.

```bash
cd ~/YOUR_ROS_WS/src
ros2 pkg create gz_example_robot_description --build-type ament_python
cd gz_example_robot_description
mkdir launch
mkdir worlds
```

Then modify the "data_files" portion of _setup.py_ file with the following:

```python
    import os
    from glob import glob

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch (launch.py) files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include world (.sdf) files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
    ],
```

We will be adding more directories later for urdf files and rviz files - so keep this step in mind.

Finally, run a _colcon build_ but skip building other packages:

```bash
cd ~/YOUR_ROS_WS/
colcon build --packages-select gz_example_robot_description
```
and source the workspace with ```source ~/YOUR_ROS_WS/install/setup.bash``` is necessary.

## Step 1 - Let's Make a World

Follow the Gazebo Sim tutorial on [making a world file](https://gazebosim.org/docs/fortress/sdf_worlds).  It will cover how to load pre-existing models from fuel (this will make sense when you have covered the tutorial), however, please do not include any fuel models in your world file.

Instead, please use [this example world file](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/examples/worlds/shapes.sdf) as inspiration, and instead include some random objects (perhaps 3 or 4).  They should not be at (0,0) (this is where we will spawn a robot later on).  Place them roughly 5 or 6 metres away.  Which objects and where are up to you.

This will be the world file we launch our robot into, typically we would just use the "empty.sdf" world if we were being lazy.

You may call the world file whatever you like, however, remember what is it, and place the file in the _/worlds_ directory of your package.

## Step 2 - Launch the World

Create a launch file called _sim_robot.launch.py_ in the launch directory.  It should be populated like so:

```python
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    pkg_name = 'gz_example_robot_description'

    # Set ignition resource path (so it can find your world files)
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
    value=[os.path.join(get_package_share_directory(pkg_name),'worlds')])

    # Include the Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
    launch_arguments={
    'gz_args' : '-r MYWORLD.sdf'
    }.items(),
    )

    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path) # Please put this before gazebo
    ld.add_action(launch_gazebo)
    return ld
```
In theory is it possible to pass the world file (MYWORLD.sdf) as an argument (with a default value) but we will skip that here to avoid any confusion and just hardcode it in.  If you don't have a world file, I would recommend just using "empty.sdf".

Don't forget to colon build and source!

Run the launch file with the usual:
```bash
ros2 launch gz_example_robot_description sim_robot.launch.py
```
Once you are content it works, go ahead and stop all the processes using ```Ctrl+C``` in the terminal like you usually would.

Did it work? ```if(works){celebrate()}else{double_check_everything()}```

## Step 3 - Simple URDF

Create a simple URDF which uses xacro to allow for variables.  The file in this instance will be called *diff_drive_simulation.urdf.xacro*, and placed in the urdf directory.

Wait a minute?  What urdf directory?!  Well, you need to make that too using ```mkdir urdf```, and add the necessary lines to your "setup.py" file.  In this case:
```python
# Include URDF (.urdf) files
(os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf*'))),
```
Note: The additional wildcard (*) after '.urdf' ensures that both _.urdf_ and _urdf.xacro_ files are visible.

Copy the following code into *diff_drive_simulation.urdf.xacro*:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="gz_example_robot">

<!-- Variables to be used in the file -->
<xacro:property name="chassis_width"    value="0.16" />
<xacro:property name="chassis_length"   value="0.24" />
<xacro:property name="chassis_height"   value="0.12" />
<xacro:property name="wheel_separation" value="0.20" />
<xacro:property name="wheel_radius"     value="0.08" />
<xacro:property name="wheel_offset"     value="${wheel_radius/4}" />
<xacro:property name="wheel_width"      value="0.04" />
<xacro:property name="sphere_radius"    value="${wheel_radius+wheel_offset-(chassis_height/2)}" />
<xacro:property name="base_offset"      value="0.08" />
<xacro:property name="footprint_z"      value="${chassis_height/2 + sphere_radius}" />

  <!-- LINKS -->
  <link name="base_footprint"/>

  <link name="base_link">
    <visual>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <origin rpy="0 0 0" xyz="${base_offset} 0 0"/>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
      </geometry>
      <origin rpy="0 0 0" xyz="${base_offset} 0 0"/>
    </collision>

    <inertial>
      <mass value="10"/>
      <inertia ixx="0.0033" ixy="0.0" ixz="0.0" iyy="0.006" iyz="0.0" izz="0.007"/>
    </inertial>
  </link>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder length="${wheel_width}" radius="${wheel_radius}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <material name="blue">
        <color rgba="0.1 0.1 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="${wheel_width}" radius="${wheel_radius}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>

    <inertial>
      <mass value="1"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.0032"/>
    </inertial>
  </link>

  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder length="${wheel_width}" radius="${wheel_radius}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <material name="blue">
        <color rgba="0.1 0.1 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="${wheel_width}" radius="${wheel_radius}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>

    <inertial>
      <mass value="1"/>
      <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.0032"/>
    </inertial>
  </link>

  <link name="front_wheel">
    <visual>
      <geometry>
        <sphere radius="${sphere_radius}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <material name="blue">
        <color rgba="0.1 0.1 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="${sphere_radius}" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 0"/>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.00016" ixy="0.0" ixz="0.0" iyy="0.00016" iyz="0.0" izz="0.00016"/>
    </inertial>
  </link>

  <link name="chassis_link">
  <!-- Convenient place to reference sensors to -->
  </link>

  <link name="imu_link">
  <!-- Frame to attach an IMU to -->
  </link> 

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder length="0.03" radius="0.03" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 -0.005"/>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder length="0.03" radius="0.03" />
      </geometry>
      <origin rpy="0 0 0" xyz="0 0 -0.005"/>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.00045"/>
    </inertial>
  </link>


  <!-- JOINTS -->
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/>
    <child link="base_link"/>
    <origin xyz="${base_offset} 0 ${footprint_z}" rpy="0 0 0"/>
  </joint>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 ${wheel_separation/2} -${wheel_offset}" rpy="-${pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -${wheel_separation/2} -${wheel_offset}" rpy="-${pi/2} 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <joint name="front_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="front_wheel"/>
    <origin xyz="${base_offset + (chassis_length/2) - sphere_radius} 0 -${chassis_height/2}" rpy="0 0 0"/>
  </joint>

  <joint name="chassis_joint" type="fixed">
    <parent link="base_link"/>
    <child link="chassis_link"/>
    <origin xyz="${base_offset} 0 ${chassis_height/2}" rpy="0 0 0"/>
  </joint>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <joint name="lidar_joint" type="fixed">
    <parent link="chassis_link"/>
    <child link="lidar_link"/>
    <origin xyz="0.1 0 0.02" rpy="0 0 0"/>
  </joint>
 
</robot>
```

Notice that links MUST have _\<interial\>_ and _\<collision\>_ tags - this makes for quite a lengthy looking file.  The interia matrix can be found online for simple geometries (sphere, cube, cylinder), or if you have a mesh file (which we won't cover today) you can find the inertia matrix from CAD software or software like [Meshlab](https://www.meshlab.net/) or [Cloudcompare](https://www.danielgm.net/cc/).  These are critical for the physics engine to handle things properly.

This is also the case for real robots!  Manipulators for example often need an accurate estimate of the centre of mass and sometimes interia matrix to handle an end-effector or payload correctly.

## Step 4 - Check Your URDF

The easiest way to check a URDF is to ensure it opens correctly in RVIZ.  Below is a convenient launch file to open the *diff_drive_simulation_urdf.xacro* file.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import xacro


def generate_launch_description():
    ld = LaunchDescription()
    

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'gz_example_robot_description'
    file_subpath = 'urdf/diff_drive_simulation.urdf.xacro'


    # Use xacro to process the file - this gets published on the /robot_description topic
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # joint state publisher (GUI) node - allows us to change non-fixed joint angles
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joint_state_publisher)
    return ld
```

Colcon build and launch the file.  Run rviz in a separate terminal with ```ros2 run rviz2 rviz2```.  Hopefully you should see your model, and you are able to manipulate the wheel joint angles.

### Making a Config File for RVIZ

It is likely you will want to have the robot model visible, as well as other topics such as TF as a sanity check.  To save you having to having to redo this everytime, a config file (.rviz) can be passed as an argument.

Make a new directory in the package called _rviz_, and add this to your setup.py file using: 

```python
# Include rviz (.rviz) files
(os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
```

When in RVIZ, get the layout how you like, go to File->Save Config As and navigate to your package in the workspace src directory, and finally to the /rviz directory.  Save the config file with a useful name.

We can add this to our *view_model.launch.py* file with the addition of:

```python
    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'view_model.rviz')]
    )

    # OTHER ld.add_action()
    # ...
    ld.add_action(node_rviz)
    return ld
```

where "view_model.rviz" is the name of the config file.  Don't forget to colcon build and source!  Even if you are being clever by using ```--symlink-install```, new files are not recognised and you have the build the ol' fashioned way regardless.

## Step 5 - Spawning the Robot Model in Gazebo

Now we are cooking, let's get the model in Gazebo.  Add these additional lines to your **sim_robot.launch.py** file.

```python
   # Specify the name of the package and path to xacro file within the package
    pkg_name = 'gz_example_robot_description'
    file_subpath = 'urdf/diff_drive_simulation.urdf.xacro'


    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath) # This line should already be in the launch file
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )

    # Run the spawner node from the ros_gz_sim package - this will hold up with new gazebo versions
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-z', '0.5'],
                        output='screen')


    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory(pkg_name), 'rviz', 'view_model.rviz')]
    )

    # OTHER ld.add_action() e.g. ign_resource_path
    # ...
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_rviz)
    return ld
```

The argument ```'-z','0.5'``` starts the robot up 0.5 metres in the air, to ensure it doesn't clip with the ground plane.

Colcon build, source and launch.  You should notice that both Gazebo and Rviz start.  The robot should be visible in both Gazebo and RVIZ, but the joints may not be correct in RVIZ.  Right now, Gazebo is not talking to ROS and vice versa - RVIZ is relying on the robot_state_publisher and /robot_description topic, which says nothing about the /joint_states of the wheels.

## Step 6 - Getting Control

Now we have a good looking URDF, we need to add things like the differential control and a joint state publisher to drive the robot around.  After the last _\<joint\>_ tag, but before _\</robot\>_ add the following code:

```xml
  <!-- GAZEBO PLUGINS -->
  <!-- Differential Drive - https://gazebosim.org/api/gazebo/4.5/classignition_1_1gazebo_1_1systems_1_1DiffDrive.html -->
  <gazebo>
    <plugin filename="ignition-gazebo-diff-drive-system" name="ignition::gazebo::systems::DiffDrive">
      <!-- Wheel Joints -->
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>

      <!-- Kinematics -->
      <wheel_separation>${wheel_separation}</wheel_separation>
      <wheel_radius>${wheel_radius}</wheel_radius>

      <odom_publish_frequency>50</odom_publish_frequency>
      
      <!-- TF Frames -->
      <frame_id>/odom</frame_id>
      <child_frame_id>/base_footprint</child_frame_id>

      <!-- topics -->
      <!-- <topic>/model/gz_example_robot/cmd_vel</topic> THIS IS DEFAULT -->
      <!-- <odom_topic>/model/gz_example_robot/odometry</odom_topic> THIS IS DEFAULT-->
      <!-- <tf_topic>/model/gz_example_robot/tf</tf_topic> THIS IS DEFAULT -->
      <!-- <tf_topic></tf_topic> Leave blank if you plan to use a filter + imu (e.g. EKF) -->
    </plugin>
  </gazebo>

  <gazebo>
      <plugin filename="ignition-gazebo-joint-state-publisher-system" name="ignition::gazebo::systems::JointStatePublisher">
        <update_rate>50</update_rate>
        <joint_name>left_wheel_joint</joint_name>
        <joint_name>right_wheel_joint</joint_name>
      </plugin>
  </gazebo>
```

The DiffDrive plugin uses the provided kinematics to turn command velocities into wheel rotation.  These command velocities are available as a gazebo topic, and we will hook into ROS in the next step.  The joint state publisher will provide topics for ROS to hook into later.

Colcon build, source the workspace, and run the gazebo sim via our handy launch file **sim_robot.launch.py**.

### Checking the Gazebo Robot

In a new terminal use the command:

```bash
ign topic --list
```
which should list all the topics internal to Gazebo.  Notice there are topics ```/model/gz_example_robot/cmd_vel```, ```/model/gz_example_robot/odometry``` and ```/world/WORLDNAME/model/gz_example_robot/joint_state```.

Try moving the robot with a terminal command: ```ign --topic --pub "linear: {x:0.1}" --topic /model/gz_example_robot/cmd_vel --msgtype ignition.msgs.Twist```

Did it work? ```if(works){celebrate()}else{double_check_everything()}```

## Step 7 - Hooking into ROS

Right now, using the terminal to send commands to the robot is pretty rubbish, we want to leverage the power of the Robot Operating Sytem!

To enable this, we need to "bridge" between gazebo and ROS.  This is acheived via the _ros_gz_bridge_ package.

In the sim_robot.launch.py launch file, add the additional commands:

```python
    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',
                    '/model/gz_example_robot/cmd_vel'  + '@geometry_msgs/msg/Twist'   + '@' + 'ignition.msgs.Twist',
                    '/model/gz_example_robot/odometry' + '@nav_msgs/msg/Odometry'     + '[' + 'ignition.msgs.Odometry',
                    '/model/gz_example_robot/tf'       + '@tf2_msgs/msg/TFMessage' + '[' + 'ignition.msgs.Pose_V',
                    '/world/MYWORLD/model/gz_example_robot/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
                    ],
        parameters= [{'qos_overrides./gz_example_robot.subscriber.reliability': 'reliable'}],
        remappings= [
                    ('/model/gz_example_robot/cmd_vel',  '/cmd_vel'),
                    ('/model/gz_example_robot/odometry', '/odom'   ),
                    ('/model/gz_example_robot/tf',       '/tf'     ),
                    ('/world/MYWORLD/model/gz_example_robot/joint_state', 'joint_states')
                    ],
        output='screen'
    )

    # REST OF CODE - PLEASE USE YOUR BRAIN

    ld.add_action(node_ros_gz_bridge)
```

The bridge can provide one-way (unidirectional) or two-way (bidirectional) sharing of topics between ROS and Gazebo.  The arguments are written as follows:

```python
'/TOPIC_NAME_IN_GZ'  + '@ROS_MSG_PKG/msg/TYPE'   + 'DIRECTION' + 'ignition.msgs.TYPE',
```
The list of mapping from ROS msg to ignition msg can be found [here](https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge).  Note that going forward, ignition.msg.Type will become gz.msg.Type for future Gazebo versions - this is likely what you might see in any very up-to-date tutorials.

The direction is declared with ```[``` ROS<-GZ (Gazbeo "publishes" into ROS), ```[``` ROS->GZ (ROS "publishes" into Gazebo), and ```@``` ROS<->GZ (both can publish into each other).  Using a unidirectional approach saves on bandwidth.

The line ```'/model/gz_example_robot/odometry' + '@nav_msgs/msg/Odometry'     + '[' + 'ignition.msgs.Odometry'```, therefore translates to a single combined string ```'/model/gz_example_robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'```, where odometry from gazebo is pushed into a ROS topic, but not the other way around.

The "remappings" decide what the topics names will be in ROS (e.g. just */cmd_vel*).

### Give It A Try

Colcon build, source and launch the simulation with the new ros bridge.  Perform a ```ros2 topic list``` and do some echoes etc to check everything is coming through properly.  Furthermore, RVIZ should be reporting the robot model accurately now (including the wheels).

Using ```ros2 run teleop_twist_keyboard teleop_twist_keyboard``` try driving the robot around (you might want to bring the linear velocity down to ~0.1 m/s).

## Step 8 - Sensors

We have a robot which can drive around... better than our static URDF, but still could do with some sensing.

### Add A Generic 2D Lidar

Add a lidar to the lidar frame with the additional .urdf.xacro code:

  ```xml
  <gazebo reference="lidar_link">
    <sensor type="gpu_lidar" name="generic_lidar_sensor">

      <topic>/model/gz_example_robot/scan</topic>
      <frame_id>lidar_link</frame_id>
      <ignition_frame_id>lidar_link</ignition_frame_id>

      <update_rate>10.0</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>512</samples>
            <resolution>1</resolution>
            <min_angle>-${pi*0.75}</min_angle>
            <max_angle>${pi*0.75}</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>10.0</max>
          <resolution>0.03</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.001</stddev>
        </noise>
      </ray>
      <always_on>1</always_on>
      <visualize>false</visualize>
    </sensor>
  </gazebo>
  <gazebo>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </gazebo>
```
This will generate a lidar scan centred about the lidar_link we defined in the URDF.  Once again we need to "bridge" the data from Gazebo to ROS:

```python
    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [ #EXISTING ARGUMENTS
                    '/model/gz_example_robot/scan'     + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
                    ],
        parameters= [{'qos_overrides./gz_example_robot.subscriber.reliability': 'reliable'}],
        remappings= [ # EXISTING ARGUMENTS
                    ('/model/gz_example_robot/scan',     '/scan'   ),
                    ],
        output='screen'
    )
```

Colcon build, source etc and fire up the simulation.  Firstly in the top right-hand corner of the Gazebo window is a vertical 3 dots icon.  Click this and search for "Visualize Lidar" - once this is enabled, you should be able to see all the rays of the lidar scan.

You should also see that they stop when they intersect one of your objects!  In Rviz, add the /scan topic and verify the scan is visible and looks sensible.

## Step 9 - Going it Alone

Now you are primed with how to add elements to your urdf file and your launch file to bridge topics.  Have a go at including an imu attached to the *imu_link* frame.

There is the official (and distinctly lacking) [tutorial](https://gazebosim.org/docs/fortress/sensors).

Don't forget to bridge the output of the topic in your launch file.

<details>
<summary><b>If you need some help...</b></summary>
 
Here is an example which has noise profiles as well.

```xml
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <plugin filename="libignition-gazebo-imu-system.so" name="ignition::gazebo::systems::Imu">
        <ros>
          <namespace></namespace>
          <remapping>~/out:=imu</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
     <imu>
     	<angular_velocity>
     	  <x>
     	    <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
              <bias_mean>0.0000075</bias_mean>
              <bias_stddev>0.0000008</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
              <bias_mean>0.0000075</bias_mean>
              <bias_stddev>0.0000008</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
              <bias_mean>0.0000075</bias_mean>
              <bias_stddev>0.0000008</bias_stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
              <bias_mean>0.1</bias_mean>
              <bias_stddev>0.001</bias_stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
              <bias_mean>0.1</bias_mean>
              <bias_stddev>0.001</bias_stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
              <bias_mean>0.1</bias_mean>
              <bias_stddev>0.001</bias_stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>
```


 </details>