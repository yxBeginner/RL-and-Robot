cmake_minimum_required(VERSION 2.8.3)
project(p2os_driver)

find_package(catkin REQUIRED COMPONENTS
  p2os_msgs
  diagnostic_updater
  nav_msgs
  roscpp
  geometry_msgs
  tf
  std_msgs
  kdl_parser
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES p2os_driver
  CATKIN_DEPENDS p2os_msgs nav_msgs roscpp geometry_msgs diagnostic_updater tf std_msgs kdl_parser
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(p2os_driver
  src/p2osnode.cc
  src/p2os.cc
  src/kinecalc.cc
  src/packet.cc
  src/robot_params.cc
  src/sip.cc
  src/p2os_ptz.cpp
)

target_link_libraries(p2os_driver ${catkin_LIBRARIES})
add_dependencies(p2os_driver p2os_msgs_gencpp)

## Mark executables and/or libraries for installation
install(TARGETS p2os_driver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
