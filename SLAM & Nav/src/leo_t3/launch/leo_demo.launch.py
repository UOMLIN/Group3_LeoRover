from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import Command, LaunchConfiguration
import xacro
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Declare package directory
    pkg_nav_demos = get_package_share_directory('leo_t3')
    model_path = os.path.join(pkg_nav_demos, 'urdf','leo_rover.urdf.xacro')
    # Use xacro to process the file
    robot_description_raw = xacro.process_file(model_path).toxml()

    rviz_config_path = os.path.join(pkg_nav_demos, 'rviz', 'leo_demo.rviz')
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        arguments=[model_path],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )   

    # Add actions to LaunchDescription
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld