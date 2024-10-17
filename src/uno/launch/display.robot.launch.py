from rclpy.node import Node
from launch import LaunchDescription
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import  RewrittenYaml

def generate_launch_description():

    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = '/opt/ros/humble/lib/'

    controller_yaml = os.path.join(
        get_package_share_directory('uno'),
        'config',
        'controller.yaml'
    )

    # load sdf
    sdf_file_path = os.path.join(
        '/home/prashun/ros2_ws/src/uno/',
        'sdf',
        'warehouse.sdf'  # Replace with the actual name of your SDF file
    )

    # model name
    pkg_project_bringup = get_package_share_directory('uno')

    # Path to your URDF file
    urdf_file_path = os.path.join(pkg_project_bringup, 'urdf', 'roberto.urdf')

    slam_config_path = os.path.join(get_package_share_directory("uno"), 'config', 'mapper.yaml')

    nav2_params_config_path = os.path.join(get_package_share_directory("uno"), 'config', 'nav2.yaml')

    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")

   # Variables
    lifecycle_nodes = ['map_saver']

    # Create our own temporary YAML files that include substitutions
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_config_path,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    
    # Read the URDF file
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
         DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
        ),
        DeclareLaunchArgument("position_x", default_value="-2.0"),
        DeclareLaunchArgument("position_y", default_value="-0.5"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        # start gazebo sim
         ExecuteProcess(
            cmd=[
                'ign', 'gazebo', str(sdf_file_path)
            ],
            output='screen',
            shell=True
        ),
        # spawn the urdf in gazebo
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-topic", "/robot_description",
                "-name", "diff_drive_robot",
                "-allow_renaming", "true",
                "-z", "0.28",
                "-x", position_x,
                "-y", position_y,
                "-Y", orientation_yaw
        ]),
        # add bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                "/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
                "/kinect/camera/color/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                "/kinect/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                "/kinect/camera/depth/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
                "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            ],
            output='screen'
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
        ),
        # jont state publisher for gui
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            arguments=[sdf_file_path],
            output=['screen']
        ),
         # robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[{'robot_description': robot_description}],
        ),
        #launch nav2 stack 
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': '/home/prashun/ros2_ws/src/uno/config/map.yaml'}]
        ),
        # Localization Node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Planner Node
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Controller Node
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Behavior Tree Node
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Map Saver
        Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[configured_params],
        ),
        # Nav2 Lifycycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
        ),
        #launch the slam toolbaox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_config_path,
                {'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'warn']
        )
    ])