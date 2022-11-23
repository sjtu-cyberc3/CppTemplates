cmake_minimum_required(VERSION 3.10)
project(cpptemplates2)

# ##########################################################
# Version
# ##########################################################
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)

# ##########################################################
# C++ Standard
# ##########################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ##########################################################
# Compiler Defines
# ##########################################################
# add_compile_definitions(VERSION_MAJOR=${VERSION_MAJOR})
# add_compile_definitions(VERSION_MINOR=${VERSION_MINOR})
# add_compile_definitions(
# WORK_SPACE_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# ##########################################################
# Compiler Options
# ##########################################################
add_compile_options(-Wall)
add_compile_options(-Werror=return-type)
# default static library for better performance
set(BUILD_SHARED_LIBS
    OFF
    CACHE BOOL "Build dynamically-linked binaries")
# configure link time optimization
option(WITH_LTO "enable link time optimization" OFF)
if(WITH_LTO)
  add_compile_options(-flto)
endif()

# ##########################################################
# Dependencies
# ##########################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      ${PROJECT_SOURCE_DIR}/cmake)
include_directories(.)

# global_defination ###
set(WORK_SPACE_PATH ${PROJECT_SOURCE_DIR})
include(${CMAKE_MODULE_PATH}/global_defination.cmake)

# glog ###
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})
link_libraries(${GLOG_LIBRARY})

# yaml ###
find_package(yaml-cpp 0.5.2 REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
link_libraries(${YAML_CPP_LIBRARIES})

# eigen ###
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# gtest ###
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# threading
find_package(Threads REQUIRED)
link_directories(Threads::Threads)

# ##########################################################
# Source files
# ##########################################################
add_subdirectory(common)
add_subdirectory(modules)
add_subdirectory(app)
add_subdirectory(ros)
