cmake_minimum_required(VERSION 2.4.6)
project(orb_imu)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#rosbuild_init()

#IF(NOT ROS_BUILD_TYPE)
#  SET(ROS_BUILD_TYPE Release)
#ENDIF()

set(CMAKE_BUILD_TYPE Release)

#MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../ORB_SLAM2/cmake_modules)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aadet
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


find_package(OpenCV REQUIRED)

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
roscpp
cv_bridge
)


include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../ORB_SLAM2
${PROJECT_SOURCE_DIR}/../ORB_SLAM2/include
${Pangolin_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../ORB_SLAM2/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../ORB_SLAM2/Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../ORB_SLAM2/lib/libORB_SLAM2.so
${catkin_LIBRARIES}
)

# Node for monocular camera
add_executable(Mono ${PROJECT_SOURCE_DIR}/src/ros_mono.cc)

add_executable(Ekf ${PROJECT_SOURCE_DIR}/src/ekf_fusion.cc)

target_link_libraries(Mono ${LIBS})

target_link_libraries(Ekf ${LIBS})


