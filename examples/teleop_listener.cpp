// PANDA CONTROLLER
#include "colors.hpp"  //ROS_STRONG_INFO
#include "data_manager.hpp"
#include "exceptions.hpp"  //PCEXC
#include "panda.hpp"

// ROS
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>



//#############################################################################
// DEFAULT VALUES ##############################################################
const auto FG_COLOR = Colors::FG_BLUE;
const auto BG_COLOR = Colors::BG_BLACK;



//#############################################################################
// GLOBAL VALUES ##############################################################
std::string NAME;
robot::Panda *panda_ptr;



//#############################################################################
// CALLBACK ###################################################################
void teleopCallback(const geometry_msgs::Vector3::ConstPtr &msg) {
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "MOVING:");
    const float X = msg->x;
    const float Y = msg->y;
    const float Z = msg->z;
    ROS_INFO_STREAM("Offset:\n"
                    << "- X: " << X << std::endl
                    << "- Y: " << Y << std::endl
                    << "- Z: " << Z << std::endl);
    try {
        auto target_pose = panda_ptr->getCurrentPose();
        target_pose.position.x += X;
        target_pose.position.y += Y;
        target_pose.position.z += Z;
        ROS_INFO_STREAM("Move to pose:\n" << target_pose);
        panda_ptr->cartesianMovement(target_pose);

    } catch (const PCEXC::panda_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }
}



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
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "START NODE: ", NAME);


    // Create class to manage the Panda arm
    ROS_INFO("INIT PANDA CONTROLLER");
    try {
        // panda_ptr.reset(new robot::Panda());
        panda_ptr = new robot::Panda();

    } catch (const PCEXC::panda_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Create subscriber
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SUBCRIPTION TO THE TOPIC: teleop");
    ros::Subscriber sub =
        node.subscribe("/panda_controller/teleop", 1000, teleopCallback);


    // Finish
    ros::waitForShutdown();
    delete panda_ptr;
    return 0;
}