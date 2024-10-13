    <!--............................... CONTROLLER .................................... -->

    <ros2_control name="IgnitionSystem" type="system">
    
        <hardware>
            <plugin>ign_ros2_control/IgnitionSystem</plugin>
        </hardware>
        <joint name="joint_chassis_front_left_wheel">
            <command_interface name="velocity"/>
            <state_interface name="position"/>
            <state_interface name="velocity"/>
        </joint>
        <joint name="joint_chassis_front_right_wheel">
            <command_interface name="velocity"/>
            <state_interface name="velocity"/>
            <state_interface name="position"/>
        </joint>
    </ros2_control>


        <!-- ........................... ROS2 CONTROL PLUGIN ................................... -->
    <gazebo>
        <plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin">
            <robot_param>robot_description</robot_param>
            <robot_param_node>robot_state_publisher</robot_param_node>
            <parameters>/home/prashun/ros2_ws/src/uno/config/controller.yaml</parameters>
        </plugin>
    </gazebo>