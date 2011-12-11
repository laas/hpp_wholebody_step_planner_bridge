cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

INCLUDE(cmake/base.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()
ADD_REQUIRED_DEPENDENCY("eigen3")
ADD_REQUIRED_DEPENDENCY("hpp-gik")
ADD_REQUIRED_DEPENDENCY("KineoController")
ADD_REQUIRED_DEPENDENCY("robotbuilder")
ADD_REQUIRED_DEPENDENCY("hpp-wholebody-step-planner")

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries

rosbuild_add_library(${PROJECT_NAME} src/pattern-generator.cc)

PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} eigen3)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-gik)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} KineoController)
PKG_CONFIG_USE_DEPENDENCY(${PROJECT_NAME} hpp-wholebody-step-planner)

#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(trajectory-writer src/trajectory-writer.cc)
#target_link_libraries(example ${PROJECT_NAME})
PKG_CONFIG_USE_DEPENDENCY(trajectory-writer KineoController)
PKG_CONFIG_USE_DEPENDENCY(trajectory-writer robotbuilder)
PKG_CONFIG_USE_DEPENDENCY(trajectory-writer hpp-wholebody-step-planner)
