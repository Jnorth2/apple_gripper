cmake_minimum_required(VERSION 3.8)
project(lfd_data_collection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(gripper_msgs REQUIRED)



add_executable(moveit_playback src/moveit_playback.cpp)


target_include_directories(moveit_playback PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(moveit_playback PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_playback
  "rclcpp"
  "moveit_ros_planning_interface"
)

install(TARGETS moveit_playback
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  lfd_data_collection/force_torque_sensor.py
  lfd_data_collection/apple_attach.py
  lfd_data_collection/optitrack.py
  lfd_data_collection/LFD.py
  DESTINATION lib/${PROJECT_NAME}
)


# install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
