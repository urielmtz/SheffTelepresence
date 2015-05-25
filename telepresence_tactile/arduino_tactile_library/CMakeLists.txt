####################################################### 
#                                                     #  
# Copyright (C) 2014                                  #
# Author: Uriel Martinez                              #
# email: uriel.marherz@gmail.com                      #
#                                                     #
#######################################################

cmake_minimum_required(VERSION 2.8)

set(LIBRARYNAME txzy)

# version number
set(txzy_VERSION_MAJOR 0)
set(txzy_VERSION_MINOR 1)

find_package(YARP)

set(CMAKE_CXX_FLAGS "-I$ENV{YARP_ROOT}/src/modules/serial -Wall -lstdc++")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib)

file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS})
add_library(${LIBRARYNAME} ${folder_source} ${folder_header})
target_link_libraries(${LIBRARYNAME} yarpmod ${CMAKE_CXX_FLAGS} ${YARP_LIBRARIES})
install(TARGETS ${LIBRARYNAME} ${LIBRARYNAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

#add_subdirectory(examples)
