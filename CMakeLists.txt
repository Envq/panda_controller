cmake_minimum_required(VERSION 2.8.3)
project(panda_controller)


set(CMAKE_CXX_STANDARD 14)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)
# find_package(Boost REQUIRED COMPONENTS filesystem)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    # INCLUDE_DIRS include
    # CATKIN_DEPENDS 
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