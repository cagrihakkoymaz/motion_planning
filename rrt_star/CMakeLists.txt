cmake_minimum_required(VERSION 3.0.2)
project(rrt_star)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  visualization_msgs
  dynamic_reconfigure
)

find_package(ompl REQUIRED)

add_service_files(
  FILES
  RRTPathFinder.srv
)

generate_dynamic_reconfigure_options(
  cfg/rrt_star.cfg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rrt_star
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs message_runtime visualization_msgs dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/rrt_star.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
