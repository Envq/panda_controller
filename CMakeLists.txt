###############################################################################
# CMAKE CONFIG ################################################################
project(panda_controller)
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED)



###############################################################################
# PACKAGES ####################################################################
# catkin
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

# json
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

# boost
# find_package(Boost REQUIRED COMPONENTS filesystem)



###############################################################################
# CATKIN CONFIG ###############################################################
catkin_package(
    # INCLUDE_DIRS include
    CATKIN_DEPENDS 
        moveit_core
        moveit_ros_planning_interface
        actionlib_msgs
)



###############################################################################
# BUILD CONFIG ################################################################
# Add include directories 
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Executables config
set(TASK1 console)
add_executable(${TASK1}  examples/${TASK1}.cpp
    src/data_manager.cpp
    src/panda.cpp)
target_link_libraries(${TASK1} ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})


set(TASK2 pick_place)
add_executable(${TASK2}  examples/${TASK2}.cpp
    src/data_manager.cpp
    src/panda.cpp)
target_link_libraries(${TASK2} ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})