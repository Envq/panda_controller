cmake_minimum_required(VERSION 2.8.3)
project(panda_controller)


set(CMAKE_CXX_STANDARD 14)


# CATKIN
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    moveit_core
    moveit_ros_planning_interface
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

add_executable(current_pose  src/current_pose.cpp
    ${MY_LIBS}/poses_manager.cpp)
target_link_libraries(current_pose ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})


add_executable(pick_place  src/pick_place.cpp
    ${MY_LIBS}/poses_manager.cpp
    ${MY_LIBS}/arm.cpp)
target_link_libraries(pick_place ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})