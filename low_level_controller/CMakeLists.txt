cmake_minimum_required(VERSION 3.8)
project(low_level_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ---- build executable ----
add_executable(pid_cascade src/PID_Cascade.cpp)
ament_target_dependencies(pid_cascade
  rclcpp
  std_msgs
  geometry_msgs
)


target_include_directories(pid_cascade PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)

# ---- install ----
install(TARGETS
  pid_cascade
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
