cmake_minimum_required(VERSION 3.8)
project(zzx_execution)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(zzx_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(fake_arm_backend src/fake_arm_backend.cpp src/execution_monitor.cpp)
target_compile_features(fake_arm_backend PUBLIC cxx_std_17)
target_include_directories(fake_arm_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(fake_arm_backend PRIVATE -Wall -Wextra -Wpedantic)
add_executable(fake_arm_node src/fake_arm_node.cpp)
target_link_libraries(fake_arm_node fake_arm_backend)
ament_target_dependencies(fake_arm_node rclcpp sensor_msgs std_srvs)
install(TARGETS fake_arm_backend EXPORT export_fake_arm_backend
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(TARGETS fake_arm_node DESTINATION lib/${PROJECT_NAME})
add_executable(move_arm_server src/move_arm_server.cpp)
target_link_libraries(move_arm_server fake_arm_backend)
target_compile_options(move_arm_server PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(move_arm_server rclcpp rclcpp_action sensor_msgs zzx_interfaces)
install(TARGETS move_arm_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_fake_arm_backend HAS_LIBRARY_TARGET)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fake_arm_backend test/test_fake_arm_backend.cpp)
  target_link_libraries(test_fake_arm_backend fake_arm_backend)
  ament_add_gtest(test_execution_monitor test/test_execution_monitor.cpp)
  target_link_libraries(test_execution_monitor fake_arm_backend)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_ros_smoke test/test_ros_smoke.py
    TIMEOUT 45 ENV ROS_DOMAIN_ID=86 ROS_LOCALHOST_ONLY=1
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp)
  ament_add_pytest_test(test_move_arm_action test/test_move_arm_action.py
    TIMEOUT 90 ENV ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp)
endif()
ament_package()
