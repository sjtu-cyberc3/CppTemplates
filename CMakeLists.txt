cmake_minimum_required(VERSION 3.5)
project(cpptemplates)

# ##############################################################################
# Version
# ##############################################################################
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)

# ##############################################################################
# C++ Standard
# ##############################################################################
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ##############################################################################
# Compiler Defines
# ##############################################################################
add_compile_options(-DWORK_SPACE_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

# ##############################################################################
# Compiler Options
# ##############################################################################
add_compile_options(-Wall)
add_compile_options(-Werror=return-type)
# default static library for better performance
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build dynamically-linked binaries")
# configure link time optimization
option(WITH_LTO "enable link time optimization" OFF)
if(WITH_LTO)
  add_compile_options(-flto)
endif()

# ##############################################################################
# Dependencies
# ##############################################################################
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
include_directories(.)

# glog ###
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})
link_libraries(${GLOG_LIBRARY})

# gtest ###
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# yaml ###
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
link_libraries(${YAML_CPP_LIBRARIES})

# fmt
find_package(fmt 9.1.0 REQUIRED)
link_libraries(fmt::fmt)

# threading
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

# ##############################################################################
# Source files
# ##############################################################################
add_subdirectory(common)
add_subdirectory(modules)
add_subdirectory(app)
add_subdirectory(ros)
