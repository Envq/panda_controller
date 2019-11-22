cmake_minimum_required(VERSION 2.8.3)
project(panda_controller)


set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
)
# find_package(Boost REQUIRED COMPONENTS filesystem)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS 
        moveit_core
        moveit_ros_planning_interface
)


###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

## My libs directory
set(MY_LIBS ${PROJECT_SOURCE_DIR}/src/my_libs)

add_executable(demo  src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES})

add_executable(obj_pose  src/obj_pose.cpp)
target_link_libraries(obj_pose ${catkin_LIBRARIES})

add_executable(current_pose  src/current_pose.cpp)
target_link_libraries(current_pose ${catkin_LIBRARIES})