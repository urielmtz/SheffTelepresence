# Copyright: (C) 2015 Sheffield Robotics - CYBERSELVES project
# Authors: Uriel Martinez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME telepresenceRobot)
project(${PROJECTNAME})
find_package(ICUB)
find_package(YARP)

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} icubmod yarpmod ${YARP_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION ${YARP_DIR}/bin)

