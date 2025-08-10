# Goal
**setup the tf for robot, arm, and camera, using geometry/Pose sent from STM32**
### current version(for group 再見機器人):
- world - map (static transform, map is where your robots start PID)
- map - robot center (dynamic transform, data given by geometry_msgs/Pose, name: /Odometry)
- robot center - lift base (dynamic transform, data given by geometry_msgs/Pose, name: /lift_base)
- lift base - arm base (dynamic transform, data given by geometry_msgs/Pose, name: /arm_base)
- arm base - arm end (dynamic transform, data given by geometry_msgs/Pose, name: /arm_end)
- arm end - camera (static transform, the data should be put in a param file, not set up yet)


side note for STM32: the orientation sending to ROS should be quaternion
