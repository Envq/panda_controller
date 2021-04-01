#include <iostream>
#include <ros/ros.h>
#include <string>

#include "panda_controller/colors.hpp"
#include "panda_controller/exceptions.hpp"


// CONFIGS ====================================================================
#define FG_COLOR Colors::FG_BLUE
#define BG_COLOR Colors::BG_DEFAULT

using namespace panda_controller;



// MAIN =======================================================================
int main(int argc, char **argv) {
    // Get the file name
    std::string file_path = argv[0];
    const std::string NAME =
        file_path.substr(file_path.find_last_of("/\\") + 1);


    // Setup ROS
    ros::init(argc, argv, NAME);
    ros::NodeHandle node("~");
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "START NODE: ", NAME);

    try {
        throw PandaControllerErr("blu()", Colors::FG_BLUE_BRIGHT);
    } catch (const PandaControllerErr &err) {
        std::cout << err.getInfo() << std::endl;
    }

    try {
        throw DataManagerErr("foo()", "err");
    } catch (const DataManagerErr &err) {
        std::cout << err.what() << std::endl;
        ROS_FATAL_STREAM(get_err_msg(NAME, err.what()));
    }



    // Finish
    ros::shutdown();
    return 0;
}