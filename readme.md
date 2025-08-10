# Goal
**setup the tf for robot, arm, and camera, using geometry/Pose sent from STM32**
### require(for our group):
- world - robot center (dynamic transform, data given by geometry_msgs/Pose /Odometry)
- robot center - lift base (dynamic transform, data given by geometry_msgs/Pose /lift_base)
- lift base - arm base (dynamic transform, data given by geometry_msgs/Pose /arm_base)
- arm base - arm end (dynamic transform, data given by geometry_msgs/Pose /arm_end)
- arm end - camera (static transform, the data should be put in a param file)

side note for STM32: the orientation sending to ROS should be quaternion



todo: there should be one more static transform between world and map. the map is the starting place of the robots, so robot_center is related to map, not world. The PID start of the robot is map.