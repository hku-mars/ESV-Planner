
cmake_minimum_required(VERSION 2.8.3)
project(grid_path_searcher)

find_package(catkin REQUIRED COMPONENTS 
    fast_svsdf
    tf2_ros
    geometry_msgs
    roscpp 
    std_msgs
    nav_msgs
    utils
    planner_algorithm
    traj_utils
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}  # Uncomment if you are creating a library
  CATKIN_DEPENDS roscpp std_msgs utils planner_algorithm fast_svsdf
)

include_directories(
    include 
    SYSTEM 
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall -fPIC") # -Wextra -Werror

add_executable(demo_node 
    src/demo_node.cpp
)

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
# both nodes include generated message headers (traj_utils/PolyTraj.h)
add_dependencies(traj_server ${catkin_EXPORTED_TARGETS})
add_dependencies(demo_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(demo_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
    utils
    fast_svsdf
    planner_algorithm_lib
)

add_executable(random_complex 
    src/random_complex_generator.cpp
)

target_link_libraries(random_complex
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Uncomment and adjust if you are creating a library
# add_library(${PROJECT_NAME}
#     src/topo_path_search.cpp
#     include/topo_path_search.h
# )
# target_link_libraries(${PROJECT_NAME}
#     ${catkin_LIBRARIES}
# )

# Uncomment if you want to install your targets
# install(
#     TARGETS ${PROJECT_NAME}
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
# install(
#     DIRECTORY include/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )