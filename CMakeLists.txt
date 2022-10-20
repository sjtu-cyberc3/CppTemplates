cmake_minimum_required(VERSION 3.1.0)
project(demo)

# ##########################################################
# Version
# ##########################################################
set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
configure_file(
  "${PROJECT_SOURCE_DIR}/config/VersionConfig.h.in"
  "${PROJECT_SOURCE_DIR}/config/VersionConfig.h")

# ##########################################################
# Compiler Options
# ##########################################################
# add_compile_options("-Wall -Werror=return-type")
# add_compile_options("$<$<CONFIG:RELEASE>:-O3>")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
                      ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing() # 开启测试

# ##########################################################
# Dependencies
# ##########################################################
set(require_third_libs "")
include_directories(".")

# global_defination ###
include(cmake_modules/global_defination.cmake)

# glog ###
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})
list(APPEND require_third_libs ${GLOG_LIBRARY})

# gtest ###
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# yaml ###
find_package(yaml-cpp REQUIRED)

# eigen ###
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Boost ###
find_package(Boost REQUIRED COMPONENTS timer)
include_directories(${Boost_INCLUDE_DIRS})
list(APPEND require_third_libs ${Boost_LIBRARIES})

# ##########################################################
# Source files
# ##########################################################
add_subdirectory(common)
add_subdirectory(modules)

# ##########################################################
# Target
# ##########################################################
set(${PROJECT_NAME}_libs "")
list(APPEND ${PROJECT_NAME}_libs)

# ##########################################################
# Display
# ##########################################################
# message(STATUS "CXX FLAGS: ${CMAKE_CXX_FLAGS}")
# message(STATUS "Third party libs: ${require_third_libs}")
# ${${PROJECT_NAME}_libs}")

# ##########################################################
# app
# ##########################################################
add_subdirectory(app)
add_subdirectory(rosWrapper)
