cmake_minimum_required(VERSION 2.8.3)

##CHANGE THIS TO YOUR PACKAGE NAME
project(svett_ost)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    ascend_msgs
    geometry_msgs
    roscpp
    actionlib
    actionlib_msgs
    mavros_msgs
)
## Includes headers in include folder
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ascend_msgs geometry_msgs roscpp
)
## Includes headers in include folder
include_directories(
include
    ${catkin_INCLUDE_DIRS}
)


#########################################################################################
#REPEAT THIS FOR ALL NODES, WITH UNIQUE node_name FOR EACH NODE
## Adds a ros node with name node_name
add_executable(
	node_name src/node_name_node.cpp 
)

add_executable(
    ai_action_client src/ai_action_client
)
add_executable(
	control_action_server src/control_action_server
)
## Adds a compilation dependency to node_name
add_dependencies(
	node_name ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)
add_dependencies(
    ai_action_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
	control_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}

)

## Links node_name together with other libraries
target_link_libraries(
	node_name ${catkin_LIBRARIES}
)
target_link_libraries(
    ai_action_client ${catkin_LIBRARIES}
)

target_link_libraries(
	control_action_server ${catkin_LIBRARIES}
)

