cmake_minimum_required(VERSION 3.0.2)
project(utils)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++14 -O3 -fPIC"  )

add_compile_options(-Wno-unused-variable)
add_compile_options(-Wno-comment)

# Prebuilt libigl archives live in lib/ at the repository root. Locate them
# relative to this package, not the workspace, so the build also works when
# the repository is cloned into an existing workspace's src/ folder.
get_filename_component(REPO_LIB_DIR "${PROJECT_SOURCE_DIR}/../../lib" ABSOLUTE)
file(GLOB mylibigllib CONFIGURE_DEPENDS "${REPO_LIB_DIR}/*.a") #libigl对应库
if(NOT mylibigllib)
  message(FATAL_ERROR "No prebuilt libigl archives (*.a) found in ${REPO_LIB_DIR}")
endif()


find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
)
include_directories(${EIGEN3_INCLUDE_DIR})
add_message_files(FILES
  debug.msg
)
generate_messages(DEPENDENCIES
  std_msgs
)
catkin_package(
   INCLUDE_DIRS include ${TORCH_INCLUDE_DIRS}
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS nodelet roscpp rospy std_msgs message_runtime pcl_ros pcl_conversions
#   DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/Shape.cpp
   src/debug_publisher.cpp
   src/raycaster.cpp
)

# add_executable(debug_sender_node src/debug_publisher.cpp)

# message(STATUS "==============myibigl==============:${mylibigllib}")
target_link_libraries(${PROJECT_NAME} PUBLIC
  ${catkin_LIBRARIES}
  "${mylibigllib}"
)

# target_link_libraries(debug_sender_node
#   ${catkin_LIBRARIES}
# )

