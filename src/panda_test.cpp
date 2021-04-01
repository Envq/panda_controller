#include <iostream>
#include <ros/ros.h>
#include <string>

#include "panda_controller/colors.hpp"
#include "panda_controller/exceptions.hpp"
#include "panda_controller/panda_arm.hpp"
#include "panda_controller/panda_gripper.hpp"


// CONFIGS ====================================================================
#define FG_COLOR Colors::FG_BLUE
#define BG_COLOR Colors::BG_DEFAULT



// USING NAMESPACE ============================================================
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
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "START NODE: ", NAME);


    // Start Task
    try {
        auto arm = PandaArm(0.5);
        auto gripper = PandaGripper(false);

        gripper.homing();

        gripper.move(0.05);

        ros::Duration(0.5).sleep();

        gripper.grasp(0.03);



    } catch (const PandaControllerErr &err) {
        ROS_FATAL_STREAM(get_err_msg(NAME, err.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}