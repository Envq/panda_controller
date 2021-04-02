// ROS and Moveit
#include <ros/ros.h>

// C++
#include <iostream>

// Custom
#include "panda_controller/exceptions.hpp"
#include "panda_controller/panda_arm.hpp"
#include "panda_controller/panda_gripper.hpp"
#include "panda_controller/panda_scene.hpp"
#include "utils/colors.hpp"



// USING NAMESPACE ============================================================
using namespace panda_controller;



// CONFIGS== ==================================================================
namespace {
auto FG_COLOR = Colors::FG_BLUE;
auto BG_COLOR = Colors::BG_DEFAULT;
}  // namespace



// MAIN =======================================================================
int main(int argc, char **argv) {
    // Get the file name
    std::string file_path = argv[0];
    const std::string NAME =
        file_path.substr(file_path.find_last_of("/\\") + 1);


    // Setup ROS
    ros::init(argc, argv, NAME);
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "START NODE: ", NAME);


    // Start Task
    try {
        auto arm = PandaArm();
        auto gripper = PandaGripper(false);
        auto scene = PandaScene();


        // Write your code here

    } catch (const PandaControllerErr &err) {
        ROS_FATAL_STREAM(get_err_msg(NAME, err.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}