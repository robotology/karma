# Copyright: (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(karmaMotor)

find_package(ICUBcontrib REQUIRED)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})

add_executable(${PROJECT_NAME} ${folder_source})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib)
install(TARGETS ${PROJECT_NAME} DESTINATION bin)

