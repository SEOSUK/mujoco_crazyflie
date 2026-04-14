cmake_minimum_required(VERSION 3.8)
project(mujoco_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(glfw3 REQUIRED)

set(MUJOCO_INCLUDE_DIR "/home/seosuk/.local/lib/python3.10/site-packages/mujoco/include")
set(MUJOCO_LIBRARY "/home/seosuk/.local/lib/python3.10/site-packages/mujoco/libmujoco.so.3.4.0")

message(STATUS "MUJOCO_INCLUDE_DIR = ${MUJOCO_INCLUDE_DIR}")
message(STATUS "MUJOCO_LIBRARY     = ${MUJOCO_LIBRARY}")

add_executable(mujoco_bridge_node
  src/mujoco_bridge.cpp
  src/mujoco_contact.cpp
  src/viewer_controls.cpp
)

target_include_directories(mujoco_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)

ament_target_dependencies(mujoco_bridge_node
  rclcpp
  ament_index_cpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

target_link_libraries(mujoco_bridge_node
  ${MUJOCO_LIBRARY}
  glfw
)

install(TARGETS
  mujoco_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
