cmake_minimum_required(VERSION 3.0.2)
project(cyphal_communicator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -Werror -std=c++17")

catkin_package(
 CATKIN_DEPENDS std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(CYPHAL_REPO_DIR ${CMAKE_CURRENT_LIST_DIR}/Libs/cyphal_application)
set(PLATFORM socketcan)
set(libparamsPath ${CMAKE_CURRENT_LIST_DIR}/Libs/libparams)
include(${CYPHAL_REPO_DIR}/CMakeLists.txt)

include_directories(${CYPHAL_REPO_DIR})
include_directories(${CYPHAL_REPO_DIR}/Udral)
include_directories(src)

add_executable(${PROJECT_NAME}
  ${CYPHAL_REPO_DIR}/Udral/actuator.cpp
  ${CYPHAL_REPO_DIR}/Udral/barometer.cpp
  ${CYPHAL_REPO_DIR}/Udral/gnss.cpp
  ${CYPHAL_REPO_DIR}/Udral/magnetometer.cpp
  ${CYPHAL_REPO_DIR}/Udral/imu.cpp

  src/main.cpp
  src/math.cpp
  src/simulator_interface/ap_json/ap_json.cpp
  src/simulator_interface/ros_interface/ros_interface.cpp
  src/autopilot_interface/cyphal_hitl.cpp
  src/params.cpp
  ${CYPHAL_SRC}
)

target_include_directories(${PROJECT_NAME}
  BEFORE PUBLIC
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${UAVCAN_LIB}
)

add_custom_target(
  compile_dsdl ALL
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/compile_dsdl.sh
  COMMENT "Compile DSDL"
)

add_custom_target(
  gen_headers ALL
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/Libs/cyphal_application/scripts/nnvg_generate_c_headers.sh
  COMMENT "Generate C headers"
)

add_dependencies(${PROJECT_NAME} compile_dsdl)
add_dependencies(${PROJECT_NAME} gen_headers)
