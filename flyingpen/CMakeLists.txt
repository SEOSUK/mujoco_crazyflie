cmake_minimum_required(VERSION 3.8)
project(flyingpen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------- dependencies ----------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---------------- executables ----------------
add_executable(trajectory8
  src/trajectory8.cpp
)
ament_target_dependencies(trajectory8
  rclcpp
  std_msgs
)

# ✅ NEW: trajectory_generation node
add_executable(trajectory_generation
  src/trajectory_generation.cpp
)
ament_target_dependencies(trajectory_generation
  rclcpp
  std_msgs
)

# ---------------- install ----------------
install(TARGETS
  trajectory8
  trajectory_generation
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- testing ----------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
