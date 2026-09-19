cmake_minimum_required(VERSION 3.8)
project(zzx_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(msg_files
  "msg/ObjectTarget.msg"
  "msg/SubsystemStatus.msg"
  "msg/TaskCommand.msg"
)

set(srv_files
  "srv/GetSystemStatus.srv"
  "srv/SetControlMode.srv"
)

set(action_files
  "action/AlignBase.action"
  "action/ControlHand.action"
  "action/ExecuteTask.action"
  "action/LocateObject.action"
  "action/MoveArm.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
