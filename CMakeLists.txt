cmake_minimum_required(VERSION 3.0.2)
project(pepper_demo_lib)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  nao_interaction_msgs
  geometry_msgs
  roscpp
  tf2_geometry_msgs
  tf2_ros
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pepper_demo_lib
  CATKIN_DEPENDS nao_interaction_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  rosbridgecpp
)
add_subdirectory(rosbridgecpp rosbridgecpp)

add_library(${PROJECT_NAME}
  src/Actions.cpp
  src/PepperRobot.cpp
  src/Task.cpp
  rosbridgecpp/rosbridge_ws_client.cpp

)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} rosbridgecpp)
add_dependencies(${PROJECT_NAME} nao_interaction_msgs_generate_messages_cpp)