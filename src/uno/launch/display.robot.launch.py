from rclpy.node import Node
from launch import LaunchDescription
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

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
        'empty_room.sdf'  # Replace with the actual name of your SDF file
    )

    # model name
    pkg_project_bringup = get_package_share_directory('uno')

    # Path to your URDF file
    urdf_file_path = os.path.join(pkg_project_bringup, 'urdf', 'diff_drive_robot.urdf')

     # load sdf
    sdf_file_path = os.path.join(
        '/home/prashun/ros2_ws/src/uno/',
        'sdf',
        'empty_room.sdf'  # Replace with the actual name of your SDF file
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("uno"),
            "config",
            "robot.rviz",
        ]
    )

    gui = LaunchConfiguration("gui")

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
        # start gazebo sim
         ExecuteProcess(
            cmd=[
                'ign', 'gazebo', str(sdf_file_path)
            ],
            output='screen',
            shell=True
        ),
        # spawn the urdf in gazebo
        ExecuteProcess(
            cmd=[
                'ign', 'service', '-s', '/world/default/create',
                '--reqtype', 'ignition.msgs.EntityFactory',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '1000',
                '--req', 'sdf_filename: "/home/prashun/ros2_ws/src/uno/urdf/diff_drive_robot.urdf", name: "diff_drive_robot.urdf"'
            ],
            output='screen'
        ),
        # add bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                       '/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                       '/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image'],
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
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[controller_yaml] 
        ),
        TimerAction(
            period=10.0,
            actions=[
              # Joint state broadcaster Controller 
             Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
            ),
             # bycicle drive controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_file],
                condition=IfCondition(gui),
            )
        ]),
    ])