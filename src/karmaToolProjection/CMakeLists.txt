# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(karmaToolProjection)
   
find_package(ICUBcontrib REQUIRED)
find_package(YARP 3.3.0 COMPONENTS os sig math cv)
find_package(ICUB REQUIRED)
find_package(OpenCV REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

include_directories(${PROJECT_SOURCE_DIR}/include)

set(folder_source main.cpp)
# Search for source code.
file(GLOB folder_source src/*.cpp )
file(GLOB folder_header include/iCub/*.h )

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# add executables and link libraries.
add_executable(${PROJECT_NAME} ${folder_header} ${folder_source})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

