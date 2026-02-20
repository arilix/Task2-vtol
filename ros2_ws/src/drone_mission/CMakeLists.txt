cmake_minimum_required(VERSION 3.5)
project(drone_mission)

# Default to C++14 (Foxy) / C++17 (Humble) - C++14 is compatible with both
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============= Find dependencies =============
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ============= C++ Executables =============

# --- mavros_control_node ---
add_executable(mavros_control_node src/mavros_control_node.cpp)
ament_target_dependencies(mavros_control_node
  rclcpp
  geometry_msgs
  mavros_msgs
  sensor_msgs
  std_msgs
)

# --- square_mission_node ---
add_executable(square_mission_node src/square_mission_node.cpp)
ament_target_dependencies(square_mission_node
  rclcpp
  geometry_msgs
  mavros_msgs
  std_msgs
)

# ============= Install C++ targets =============
install(TARGETS
  mavros_control_node
  square_mission_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============= Install Python scripts =============
install(PROGRAMS
  scripts/mavros_control.py
  scripts/square_mission.py
  DESTINATION lib/${PROJECT_NAME}
)

# ============= Install launch files =============
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ============= Testing =============
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
