
# to start robot

 ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.12, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 1.0}}'
