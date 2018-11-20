##          Copyright Emil Fresk 2016 - 2017
## Distributed under the Boost Software License, Version 1.0.
##    (See accompanying file LICENSE_1_0.txt or copy at
##          http://www.boost.org/LICENSE_1_0.txt)

cmake_minimum_required(VERSION 2.8.3)

########################################
# Project name
########################################
project(kfly_comm)
find_package(catkin QUIET)

########################################
# Enable C++14 and other C++ flags
########################################
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")

########################################
# catkin requirements
########################################

if (catkin_FOUND)
    catkin_package(
        DEPENDS pthread
        INCLUDE_DIRS ${catkin_INCLUDE_DIRS} include lib
        LIBRARIES ${PROJECT_NAME}
    )
endif()

########################################
# Library linking and source
########################################
include_directories(${catkin_INCLUDE_DIRS}
                    include
                    lib)

add_library(${PROJECT_NAME} STATIC
            src/kfly_comm.cpp)


if (catkin_FOUND)
    add_dependencies(${PROJECT_NAME}
                     ${${PROJECT_NAME}_EXPORTED_TARGETS}
                     ${catkin_EXPORTED_TARGETS})
endif ()

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES})

########################################
# Include the example in the build
########################################
add_subdirectory(example)

