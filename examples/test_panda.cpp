// PANDA CONTROLLER
#include "colors.hpp"  //ROS_STRONG_INFO
#include "data_manager.hpp"
#include "exceptions.hpp"  //PCEXC
#include "panda.hpp"

// ROS
#include <ros/ros.h>



//##############################################################################
// DEFAULT VALUES ##############################################################
const auto FG_COLOR = Colors::FG_BLUE;
const auto BG_COLOR = Colors::BG_BLACK;



//#############################################################################
// MAIN #######################################################################
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
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "START NODE: ", NAME);


    try {
        // moveit::planning_interface::MoveGroupInterface hand("hand");
        // std::vector<double> joints;
        // joints.resize(2);
        // // hand.setNamedTarget("close");
        // joints[0] = 0.0;
        // joints[1] = 0.0;
        // hand.setJointValueTarget(joints);
        // hand.move();
        // ros::WallDuration(1.0);
        // // hand.setNamedTarget("open");
        // joints[0] = 0.035;
        // joints[1] = 0.035;
        // hand.setJointValueTarget(joints);
        // hand.move();
        // ros::WallDuration(1.0);


        auto panda = robot::Panda(false);
        panda.gripperMove(0.0);
        ros::WallDuration(2.0);
        panda.gripperMove(robot::config::GRIPPER_MAX_WIDTH);
        ros::WallDuration(2.0);


        // std::string STR;
        // node.getParam("link", STR);
        // moveit::planning_interface::MoveGroupInterface
        // panda("panda_arm_hand"); ros::WallTime(2.0); std::cout << "eef link:"
        // << panda.getEndEffectorLink() << std::endl; std::cout << "eef name:"
        // << panda.getEndEffector() << std::endl;

        // for (auto var : panda.getLinkNames()) {
        //     std::cout << var << std::endl;
        // }

        // ROS_WARN_STREAM(STR);
        // panda.setPoseTarget(data_manager::get_pose("test"), STR);
        // panda.move();

    } catch (const PCEXC::PandaControllerException &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }



    // Finish
    ros::shutdown();
    return 0;
}