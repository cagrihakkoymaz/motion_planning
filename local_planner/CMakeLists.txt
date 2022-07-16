cmake_minimum_required(VERSION 3.0.2)
project(local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  rrt_star
  tf
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES local_planner
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs tf rrt_star
#  DEPENDS system_lib
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/local_planner.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )