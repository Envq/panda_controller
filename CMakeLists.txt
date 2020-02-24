###############################################################################
# CMAKE CONFIG ################################################################
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
    message_generation
)

# boost
# find_package(Boost REQUIRED COMPONENTS filesystem)



###############################################################################
# MSGS ########################################################################
add_message_files(
    FILES
    teleop_panda.msg
  )

generate_messages(
    DEPENDENCIES
    std_msgs
)



###############################################################################
# CATKIN CONFIG ###############################################################
catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES panda_controller
    CATKIN_DEPENDS 
        moveit_core
        moveit_ros_planning_interface
        actionlib_msgs
        std_msgs
        message_runtime
)


###############################################################################
# BUILD CONFIG ################################################################
# Add include directories 
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Executables config
set(TASK console)
add_executable(${TASK}  examples/${TASK}.cpp
    src/data_manager.cpp
    src/colors.cpp
    src/panda.cpp)
target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)


set(TASK pick_place)
add_executable(${TASK}  examples/${TASK}.cpp
    src/data_manager.cpp
    src/colors.cpp
    src/panda.cpp)
target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)


# set(TASK teleop_listener)
# add_executable(${TASK}  examples/${TASK}.cpp
#     src/data_manager.cpp
#     src/colors.cpp
#     src/panda.cpp)
# add_dependencies(${TASK} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)


# set(TASK teleop_talker)
# add_executable(${TASK}  examples/${TASK}.cpp
#     src/data_manager.cpp
#     src/colors.cpp
#     src/panda.cpp)
# add_dependencies(${TASK} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)


set(TASK test_panda)
add_executable(${TASK}  examples/${TASK}.cpp
    src/data_manager.cpp
    src/colors.cpp
    src/panda.cpp)
add_dependencies(${TASK} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${TASK} ${catkin_LIBRARIES} yaml-cpp)