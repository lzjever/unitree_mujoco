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

enable_testing()

set(PATCHED_SIMULATE_DIR "${CMAKE_CURRENT_BINARY_DIR}/patched_mujoco_simulate")
file(MAKE_DIRECTORY "${PATCHED_SIMULATE_DIR}")

file(READ "${MUJOCO_DIR}/simulate/simulate.h" PATCHED_SIMULATE_H)
string(REPLACE
  "#include <condition_variable>\n"
  "#include <condition_variable>\n#include <functional>\n"
  PATCHED_SIMULATE_H
  "${PATCHED_SIMULATE_H}")
string(REPLACE
  "  mjvScene* user_scn = nullptr;\n"
  "  mjvScene* user_scn = nullptr;\n  std::function<void(mjvScene*)> scene_overlay_callback;\n"
  PATCHED_SIMULATE_H
  "${PATCHED_SIMULATE_H}")
file(WRITE "${PATCHED_SIMULATE_DIR}/simulate.h" "${PATCHED_SIMULATE_H}")

file(READ "${MUJOCO_DIR}/simulate/simulate.cc" PATCHED_SIMULATE_CC)
string(REPLACE
  "      if (!is_passive_) {\n        Sync();\n      } else if (m_passive_ && d_passive_) {"
  "      if (!is_passive_) {\n        Sync();\n        if (scene_overlay_callback) {\n          scene_overlay_callback(&this->scn);\n        }\n      } else if (m_passive_ && d_passive_) {"
  PATCHED_SIMULATE_CC
  "${PATCHED_SIMULATE_CC}")
string(REPLACE
  "        std::memcpy(this->scn.geoms + this->scn.ngeom, user_scn_geoms_.data(),\n                    ngeom * sizeof(mjvGeom));\n        this->scn.ngeom += ngeom;\n      }"
  "        std::memcpy(this->scn.geoms + this->scn.ngeom, user_scn_geoms_.data(),\n                    ngeom * sizeof(mjvGeom));\n        this->scn.ngeom += ngeom;\n        if (scene_overlay_callback) {\n          scene_overlay_callback(&this->scn);\n        }\n      }"
  PATCHED_SIMULATE_CC
  "${PATCHED_SIMULATE_CC}")
file(WRITE "${PATCHED_SIMULATE_DIR}/simulate.cc" "${PATCHED_SIMULATE_CC}")

link_directories(${MUJOCO_DIR}/lib)

set(SIM_INCLUDE_DIRS
  ${MUJOCO_DIR}/include
  ${MUJOCO_DIR}/simulate
  src/lodepng
)

file(GLOB SIM_COMMON_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  ${MUJOCO_DIR}/simulate/glfw_*.cc
  ${MUJOCO_DIR}/simulate/platform_*.cc
  src/lodepng/lodepng.cpp
)

set(SIM_SRC
  ${SIM_COMMON_SRC}
  ${MUJOCO_DIR}/simulate/simulate.cc
)

set(GHOST_SIM_SRC
  ${SIM_COMMON_SRC}
  ${PATCHED_SIMULATE_DIR}/simulate.cc
)

set(REF_GHOST_SRC
  src/reference_frame_cache.cc
  src/reference_frame_client.cc
  src/ghost_scene_overlay.cc
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
target_include_directories(unitree_mujoco PRIVATE ${SIM_INCLUDE_DIRS})

add_executable(unitree_mujoco_agentic_ghost ${GHOST_SIM_SRC} ${REF_GHOST_SRC} src/main.cc)
target_include_directories(unitree_mujoco_agentic_ghost PRIVATE
  ${PATCHED_SIMULATE_DIR}
  ${SIM_INCLUDE_DIRS}
)
target_compile_definitions(unitree_mujoco_agentic_ghost PRIVATE
  UNITREE_MUJOCO_GHOST_VIEWER=1
  UNITREE_MUJOCO_GHOST_DEFAULT_ENABLED=1
)

add_executable(reference_frame_selftest
  src/reference_frame_selftest.cc
  src/reference_frame_cache.cc
  src/reference_frame_client.cc
  src/ghost_scene_overlay.cc
)
target_include_directories(reference_frame_selftest PRIVATE ${SIM_INCLUDE_DIRS})
add_test(NAME reference_frame_selftest COMMAND reference_frame_selftest)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
target_include_directories(jstest PRIVATE ${SIM_INCLUDE_DIRS})
