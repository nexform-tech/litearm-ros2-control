cmake_minimum_required(VERSION 3.16)
project(litearm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointParam.msg"
  "msg/LitearmStatus.msg"

  "srv/ActivateLicense.srv"
  "srv/GetFeedforwardScalar.srv"
  "srv/GetJointParams.srv"
  "srv/GetLicense.srv"
  "srv/GetStatus.srv"
  "srv/SetFeedforwardMask.srv"
  "srv/SetFeedforwardPreset.srv"
  "srv/SetFeedforwardScalar.srv"
  "srv/SetFeedforwardVector.srv"
  "srv/SetJointGains.srv"
  "srv/SetJointLimits.srv"
  "srv/SetMotionMode.srv"
  "srv/SetPayload.srv"
  "srv/SetSpeedScaling.srv"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
