from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
def generate_launch_description():
    ld = LaunchDescription()

    # Declare package directory
    pkg_nav_demos = get_package_share_directory('navigation_demos')

    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # List of nodes in order of launch
    lifecycle_nodes = [
        'amcl',
        'planner_server',
        'controller_server',
        'behaviour_server',
        'bt_navigator'
        # 'waypoint_follower'
    ]




    # LOAD PARAMETERS FROM YAML FILES
    # config_bt_nav = PathJoinSubstitution([pkg_nav_demos, 'config', 'bt_nav.yaml'])
    config_nav2params = PathJoinSubstitution([pkg_nav_demos, 'config', 'nav2_params.yaml'])
    twist_mux_params = PathJoinSubstitution([pkg_nav_demos,'config','twist_mux.yaml'])


    # Include Gazebo Simulation
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gz_example_robot_description'), '/launch', '/sim_robot.launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # Include SLAM Toolbox standard launch file
    launch_slamtoolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )
    # Include Joystick Launch File
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('navigation_demos'),'/launch','/joystick.launch.py']), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include RPLidar Launch File
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('rplidar_ros'),'/launch','/view_rplidar_a2m12_launch.py']), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Twist Mux Node
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )
    # Behaviour Tree Navigator
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[config_nav2params],
        remappings=remappings,
    )

    # Behaviour Tree Server (Recovery Nodes renamed to Behaviour Nodes)
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[config_nav2params],
        remappings=remappings,
    )
    # # Waypoint Node
    # node_waypoint = Node(
    #     package='nav2_waypoint_follower',
    #     executable='waypoint_follower',
    #     name='waypoint_follower',
    #     output='screen',
    #     parameters=[config_nav2params],
    #     remappings=remappings,
    # )
    
    # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )

    # Planner Server Node
    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config_nav2params],
        remappings=remappings,
    )

    # Controller Server Node
    node_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[config_nav2params],
        remappings=remappings,
    )

    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[config_nav2params],
        remappings=remappings,
    )
    # node_map_server = Node(
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{config_nav2params}],
    # )
    node_local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[{config_nav2params}],
    )
    
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_gazebo)
    ld.add_action(launch_slamtoolbox)
    ld.add_action(twist_mux)
    ld.add_action(joystick)
    ld.add_action(lidar)
    # bt_nav, behaviour, planner, controller, lifecycle_manager- ORDER MATTERS
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)
    ld.add_action(node_planner)
    ld.add_action(node_controller)
    ld.add_action(node_local_costmap)  
    ld.add_action(node_amcl)

    # ld.add_action(node_waypoint)
    
    ld.add_action(node_lifecycle_manager)

    return ld