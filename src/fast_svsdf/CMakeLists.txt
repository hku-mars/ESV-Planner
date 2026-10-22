cmake_minimum_required(VERSION 3.0.2)
project(fast_svsdf)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall --std=c++14 -O3 -fPIC")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
    roscpp 
    std_msgs
    map_manager
    utils
    visualization_pkg
    planner_algorithm
    traj_utils
)

if(NOT DEFINED LIBIGL_INCLUDE_DIR)
  set(LIBIGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../utils/include)
endif()

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBIGL_INCLUDE_DIR}
)

# Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp map_manager utils visualization_pkg planner_algorithm traj_utils
)

# Declare the library target
add_library(${PROJECT_NAME} STATIC
   src/planner_strategy.cpp
   src/planner_manager.cpp
)
# planner_manager.hpp includes generated message headers (traj_utils/PolyTraj.h)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

# Specify libraries to link against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  map_manager_lib
  visualization_pkg
  planner_algorithm_lib
)

