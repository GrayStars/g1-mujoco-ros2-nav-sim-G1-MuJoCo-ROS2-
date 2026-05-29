cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

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

set(MUJOCO_SDK_DIR "$ENV{MUJOCO_DIR}")
if(DEFINED MUJOCO_DIR)
  set(MUJOCO_SDK_DIR "${MUJOCO_DIR}")
endif()
if(NOT MUJOCO_SDK_DIR)
  set(MUJOCO_SDK_DIR "/home/xkh/.mujoco/mujoco-3.2.7")
endif()

include_directories(
  /usr/local/include/iceoryx/v2.0.2
  ${MUJOCO_SDK_DIR}/include
  ${MUJOCO_SDK_DIR}/simulate
  ${MUJOCO_SDK_DIR}/include/mujoco
  mujoco/include
  mujoco/simulate
  src/lodepng
)
link_directories(
  ${MUJOCO_SDK_DIR}/lib
  mujoco/lib
)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco/simulate/main.cc
  ${MUJOCO_SDK_DIR}/simulate/glfw_*.cc
  ${MUJOCO_SDK_DIR}/simulate/platform_*.cc
  ${MUJOCO_SDK_DIR}/simulate/simulate.cc
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
  rt
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_sources(unitree_mujoco PRIVATE src/shm_nav_bridge.cc)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
