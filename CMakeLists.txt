# CMAKE CONFIG ================================================================
cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED)
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")
project(panda_controller)

# Compile options
add_compile_options("-Wall")
# add_compile_options("-Wextra")
# add_compile_options("-Wpedantic")
# add_compile_options("-Werror")


# PACKAGES ====================================================================
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_generation
    std_msgs
    geometry_msgs
    shape_msgs
    actionlib
    actionlib_msgs
    moveit_core
    moveit_ros_planning_interface
    franka_gripper
)

# yaml
find_package(yaml-cpp REQUIRED)

# boost
# find_package(Boost REQUIRED COMPONENTS filesystem)


# MSGS ========================================================================
add_message_files(
    FILES
    panda_teleop.msg
  )

generate_messages(
    DEPENDENCIES
    std_msgs
)


# CATKIN ======================================================================
catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES panda_controller
    CATKIN_DEPENDS 
        std_msgs
        message_runtime
        moveit_core
        moveit_ros_planning_interface
        actionlib_msgs
)


# BUILD =======================================================================
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAML_INCLUDE_DIRS}
)

SET(SOURCES
    ${PROJECT_SOURCE_DIR}/src/utils/colors.cpp
    ${PROJECT_SOURCE_DIR}/src/panda_controller/data_manager.cpp
    ${PROJECT_SOURCE_DIR}/src/panda_controller/panda_arm.cpp
    ${PROJECT_SOURCE_DIR}/src/panda_controller/panda_gripper.cpp
    ${PROJECT_SOURCE_DIR}/src/panda_controller/panda.cpp
)

set(TASK console)
add_executable(${TASK} src/${TASK}.cpp ${SOURCES})
target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)

set(TASK pick_and_place)
add_executable(${TASK} src/${TASK}.cpp ${SOURCES})
target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)

set(TASK teleop_talker)
add_executable(${TASK} src/${TASK}.cpp ${SOURCES})
add_dependencies(${TASK} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)

set(TASK teleop_listener)
add_executable(${TASK} src/${TASK}.cpp ${SOURCES})
add_dependencies(${TASK} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)

set(TASK panda_test)
add_executable(${TASK} src/${TASK}.cpp ${SOURCES})
target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)