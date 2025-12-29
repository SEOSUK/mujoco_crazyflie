cmake_minimum_required(VERSION 3.8)
project(flyingpen_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Curses REQUIRED)                 # ★ 추가 (ncurses)

# ---------------- Executables ----------------

# data_logger node
add_executable(data_logger src/data_logging.cpp)
ament_target_dependencies(data_logger
  rclcpp
  geometry_msgs
  std_msgs
)
target_link_libraries(data_logger Eigen3::Eigen)

install(TARGETS
  data_logger
  DESTINATION lib/${PROJECT_NAME}
)

# command_publisher node
add_executable(command_publisher src/command_publisher.cpp)  # ★ 추가
ament_target_dependencies(command_publisher
  rclcpp
  std_msgs
)
target_include_directories(command_publisher PRIVATE ${CURSES_INCLUDE_DIR}) # ★ 추가
target_link_libraries(command_publisher ${CURSES_LIBRARIES})               # ★ 추가

install(TARGETS
  command_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------- Install resources ----------------

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
