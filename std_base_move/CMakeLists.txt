cmake_minimum_required(VERSION 2.8.3)
project(std_base_move)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs 
    tf2
    tf2_ros
    geometry_msgs
    message_generation 
    sensor_msgs 
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   vel_pub.msg 
)

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs 
)


catkin_package(
    CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    message_runtime
    tf2
    tf2_ros
)

set (DRIVER_PATH "./driver")

FILE(GLOB DRIVER_SRC 
    "${DRIVER_PATH}/src/impl/unix/*.cpp"
    "${DRIVER_PATH}/src/*.cpp"
)

include_directories(
    ./include
    ./driver/src
    ./driver/include
    ${catkin_INCLUDE_DIRS}
)


 add_executable(base_move_node src/base_move_node.cpp ${DRIVER_SRC})
 add_executable(ydlidar_node src/ydlidar_node.cpp ${DRIVER_SRC})

 add_dependencies(base_move_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 add_dependencies(ydlidar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(base_move_node ${catkin_LIBRARIES})
 target_link_libraries(ydlidar_node ${catkin_LIBRARIES})
