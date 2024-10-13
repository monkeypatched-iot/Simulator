    # for real robot add 

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


    <!-- ........................... KINECT SENSOR................................... -->
    <sensor type="depth" name="kinect_camera">
            <camera name="cam">
                <save enabled="true">
                    <path>/tmp/camera_save_tutorial</path>
                </save>
                <horizontal_fov>1.3962634</horizontal_fov>
                <image>
                    <width>640</width>
                    <height>480</height>
                    <format>R8B8G8</format>
                </image>
                <clip>
                    <near>0.02</near>
                    <far>300</far>
                </clip>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.007</stddev>
                </noise>
            </camera>
        </sensor>
        <plugin name="camera_plugin" filename="libgazebo_ros_openni_kinect.so">
            <baseline>0.2</baseline>
            <alwaysOn>true</alwaysOn>
            <!-- Keep this zero, update_rate in the parent <sensor> tag
                will control the frame rate. -->
            <updateRate>20.0</updateRate>
            <cameraName>camera_ir</cameraName>
            <imageTopicName>/kinect/camera/color/image_raw</imageTopicName>
            <cameraInfoTopicName>/kinect/camera/color/camera_info</cameraInfoTopicName>
            <depthImageTopicName>/kinect/camera/depth/image_raw</depthImageTopicName>
            <depthImageCameraInfoTopicName>/kinect/camera/depth/camera_info</depthImageCameraInfoTopicName>
            <pointCloudTopicName>/kinect/camera/depth/points</pointCloudTopicName>
            <frameName>depth_camera_link</frameName>
            <pointCloudCutoff>0.05</pointCloudCutoff>
            <distortionK1>0</distortionK1>
            <distortionK2>0</distortionK2>
            <distortionK3>0</distortionK3>
            <distortionT1>0</distortionT1>
            <distortionT2>0</distortionT2>
            <CxPrime>0</CxPrime>
            <Cx>0</Cx>
            <Cy>0</Cy>
            <focalLength>0</focalLength>
            <hackBaseline>0</hackBaseline>
            </plugin>