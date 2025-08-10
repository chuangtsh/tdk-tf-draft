# Goal
**setup the tf for robot, arm, and camera, using geometry/Pose sent from STM32**
### current version(for group 再見機器人):
- world - map
 - (static transform, map is where your robots start PID 還沒弄鏡像場地)
- map - robot center
  - (dynamic transform, name: /Odometry)
- robot center - lift base
  - (dynamic transform, name: /lift_base)
- lift base - arm base
  - (dynamic transform, name: /arm_base)
- arm base - arm end
  - (dynamic transform, name: /arm_end)
- arm end - camera
  - (static transform, the data should be put in a .param, not set up yet)


十分推薦用AI寫tf，效率極高
