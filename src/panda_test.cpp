#include <iostream>
#include <ros/ros.h>
#include <string>

#include "panda_controller/colors.hpp"



// DEFAULT VALUES ==============================================================
const auto FG_COLOR = Colors::FG_BLUE;
const auto BG_COLOR = Colors::BG_BLACK;



// MAIN =======================================================================
int main(int argc, char **argv) {
    // Get the file name
    std::string file_path = argv[0];
    const std::string NAME =
        file_path.substr(file_path.find_last_of("/\\") + 1);


    // Setup ROS
    ros::init(argc, argv, NAME);
    ros::NodeHandle node("~");
    ROS_FCOL_INFO(Colors::FG_WHITE, Colors::BG_RED, "START NODE: ", NAME);
    ROS_COL_INFO(Colors::FG_RED, "START NODE: ", NAME, 12, 130, "ciao");
    ROS_PRINT(Colors::FG_GREEN, "DEBUG", "START NODE: ", NAME, 12, 130, "ciao");



    // Finish
    ros::shutdown();
    return 0;
}