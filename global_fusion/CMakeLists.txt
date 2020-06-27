cmake_minimum_required(VERSION 2.8.3)
project(global_fusion)

set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Gflags)
find_package(Glog)
include_directories(${GFLAGS_INCLUDE_DIRS} ${GLOG_INCLUDE_DIRS})
include_directories(include)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nlosExclusion
)

find_package(Ceres REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)


catkin_package()


add_executable(spp_node 
	src/spp.cpp)
target_link_libraries(spp_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} 
						) 
