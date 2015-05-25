# Copyright: (C) 2011 EFAA Consortium
# Authors: Uriel Martinez
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME telepresenceTactile)
project(${PROJECTNAME})

find_package(YARP)

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)

set(CMAKE_CXX_FLAGS "-I$ENV{YARP_ROOT}/src/modules/serial -I/home/uriel/Documents/UoS/arduino_tactile/arduino_tactile_library/include -L/usr/local/lib -lstdc++")

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} yarpmod ${CMAKE_CXX_FLAGS} ${YARP_LIBRARIES} /usr/local/lib/libtxzy.a) 
install(TARGETS ${PROJECTNAME} DESTINATION bin)
