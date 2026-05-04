cmake_minimum_required(VERSION 3.8)
project(plant)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

cmake_policy(SET CMP0072 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenGL REQUIRED)

execute_process(
  COMMAND python3 -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
  OUTPUT_VARIABLE MUJOCO_PYTHON_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT MUJOCO_PYTHON_DIR)
  message(FATAL_ERROR "Could not locate the Python mujoco package.")
endif()

set(MUJOCO_INCLUDE_DIR "${MUJOCO_PYTHON_DIR}/include")
set(MUJOCO_LIB "${MUJOCO_PYTHON_DIR}/libmujoco.so.3.4.0")

if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo headers not found at ${MUJOCO_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${MUJOCO_LIB}")
  message(FATAL_ERROR "MuJoCo shared library not found at ${MUJOCO_LIB}")
endif()

add_executable(plant src/plant.cpp)
ament_target_dependencies(plant
  rclcpp
  std_msgs
  geometry_msgs
  ament_index_cpp
)

target_include_directories(plant PRIVATE
  ${MUJOCO_INCLUDE_DIR}
)

target_link_libraries(plant
  ${MUJOCO_LIB}
  glfw
  OpenGL::GL
)

set_target_properties(plant PROPERTIES
  BUILD_RPATH "${MUJOCO_PYTHON_DIR}"
  INSTALL_RPATH "${MUJOCO_PYTHON_DIR}"
)

install(TARGETS
  plant
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
