cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

set(MUJOCO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/mujoco")

if(NOT EXISTS "${MUJOCO_DIR}/include/mujoco/mujoco.h")
  message(FATAL_ERROR
    "MuJoCo headers were not found at ${MUJOCO_DIR}. "
    "Download a MuJoCo release and create the symlink expected by this project:\n"
    "  cd ${CMAKE_CURRENT_SOURCE_DIR}\n"
    "  ln -s ~/.mujoco/mujoco-3.3.6 mujoco")
endif()

if(NOT EXISTS "${MUJOCO_DIR}/simulate/simulate.h")
  message(FATAL_ERROR
    "MuJoCo simulate sources were not found at ${MUJOCO_DIR}/simulate. "
    "Make sure the mujoco symlink points to a full MuJoCo release directory.")
endif()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)


include_directories(
  ${MUJOCO_DIR}/include
  ${MUJOCO_DIR}/simulate
  src/lodepng
)
link_directories(${MUJOCO_DIR}/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  ${MUJOCO_DIR}/simulate/glfw_*.cc
  ${MUJOCO_DIR}/simulate/platform_*.cc
  ${MUJOCO_DIR}/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
