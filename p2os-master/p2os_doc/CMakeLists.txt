cmake_minimum_required(VERSION 2.8.3)
project(p2os_doc)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

install(DIRECTORY 
   p2os_driver
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )