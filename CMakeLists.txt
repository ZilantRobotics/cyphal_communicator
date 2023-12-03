cmake_minimum_required(VERSION 3.0.2)
project(cyphal_communicator)


# By default build as ROS-package. If SIM_INTERFACE is specified, build as not ROS-package
if(DEFINED ARDUPILOT_JSON_SIM_INTERFACE)
  ADD_DEFINITIONS(-DARDUPILOT_JSON_SIM_INTERFACE)
  set(SIM_INTERFACE_SRC src/simulator_interface/ap_json/ap_json.cpp)
else()
  set(SIM_INTERFACE_SRC src/simulator_interface/ros_interface/ros_interface.cpp)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
  )
  catkin_package(
    CATKIN_DEPENDS std_msgs
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
endif()

# set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -pedantic -Werror -std=c++17")
set(CYPHAL_REPO_DIR ${CMAKE_CURRENT_LIST_DIR}/Libs/cyphal_application)
set(PLATFORM socketcan)
set(libparamsPath ${CMAKE_CURRENT_LIST_DIR}/Libs/libparams)
include(${CYPHAL_REPO_DIR}/CMakeLists.txt)

include_directories(${CYPHAL_REPO_DIR})
include_directories(${CYPHAL_REPO_DIR}/Udral)
include_directories(src)

add_executable(${PROJECT_NAME}
  ${CYPHAL_REPO_DIR}/Udral/actuator.cpp
  ${CYPHAL_REPO_DIR}/Udral/airspeed.cpp
  ${CYPHAL_REPO_DIR}/Udral/battery.cpp
  ${CYPHAL_REPO_DIR}/Udral/barometer.cpp
  ${CYPHAL_REPO_DIR}/Udral/gnss.cpp
  ${CYPHAL_REPO_DIR}/Udral/magnetometer.cpp
  ${CYPHAL_REPO_DIR}/Udral/imu.cpp
  ${CYPHAL_REPO_DIR}/Udral/rangefinder.cpp
  ${CYPHAL_REPO_DIR}/Udral/rgbled.cpp

  src/main.cpp
  src/math.cpp
  ${SIM_INTERFACE_SRC}
  src/autopilot_interface/cyphal_hitl.cpp
  src/params.cpp
  ${CYPHAL_SRC}
)

target_include_directories(${PROJECT_NAME}
  BEFORE PUBLIC
)

if(NOT DEFINED ARDUPILOT_JSON_SIM_INTERFACE)
  add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )
endif()

execute_process(
  COMMAND git rev-parse --short=16 HEAD
  COMMAND_ERROR_IS_FATAL ANY
  OUTPUT_VARIABLE GIT_HASH_64_BIT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if("GIT_HASH_64_BIT" STREQUAL "")
  set(GIT_HASH_64_BIT "BADC0FFEEFFF")
endif()
set(GIT_HASH "0x${GIT_HASH_64_BIT}")
add_definitions(-DGIT_HASH=${GIT_HASH})

# Generate DSDL when build
add_custom_target(
  compile_dsdl ALL
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/compile_dsdl.sh
  COMMENT "Compile DSDL"
  OUTPUT_QUIET
)
add_custom_target(
  gen_headers ALL
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/Libs/cyphal_application/scripts/nnvg_generate_c_headers.sh
  COMMENT "Generate C headers"
  OUTPUT_QUIET
)
add_dependencies(${PROJECT_NAME} compile_dsdl)
add_dependencies(${PROJECT_NAME} gen_headers)
