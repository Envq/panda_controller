cmake_minimum_required(VERSION 2.8.3)
project(panda_controller)


set(CMAKE_CXX_STANDARD 14)


# CATKIN
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    shape_msgs
    actionlib_msgs
    actionlib
    moveit_core
    moveit_ros_planning_interface
    franka_gripper
)


# JSON
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)


# BOOST
# find_package(Boost REQUIRED COMPONENTS filesystem)



###################################
## catkin specific configuration ##
###################################
catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS 
        moveit_core
        moveit_ros_planning_interface
        actionlib_msgs
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

set(TASK1_NAME console)
add_executable(${TASK1_NAME}  src/${TASK1_NAME}.cpp
    ${MY_LIBS}/data_manager.cpp
    ${MY_LIBS}/panda.cpp)
target_link_libraries(${TASK1_NAME} ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})


set(TASK2_NAME pick_place)
add_executable(${TASK2_NAME}  src/${TASK2_NAME}.cpp
    ${MY_LIBS}/data_manager.cpp
    ${MY_LIBS}/panda.cpp)
target_link_libraries(${TASK2_NAME} ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})