cmake_minimum_required(VERSION 2.8.3)
project(seed_ros_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  controller_manager
  hardware_interface
  roscpp
  seed_solutions_sdk
)

# add c++ dependencies :
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c+11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCXX11_SUPPORTED)
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  add_definitions(-DCXX11_SUPPORTED)
else()
  message(FATAL "c++11 required but not supported")
endif()

# add boost dependencies :
find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)
if(NOT Boost_INCLUDE_DIRS)
  message(FATAL "boost required but not supported")
else()
  include_directories(include ${Boost_INCLUDE_DIRS})
endif()

catkin_package(
  LIBRARIES robot_interface
  CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    control_toolbox
    hardware_interface
    transmission_interface
    joint_limits_interface
    angles
    urdf
    seed_solutions_sdk
)

include_directories(  ${catkin_INCLUDE_DIRS})

add_executable(seed_ros_controller_node 
  src/seed_ros_controller.cpp 
  src/seed_robot_hardware.cpp
)
add_dependencies(seed_ros_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(seed_ros_controller_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
