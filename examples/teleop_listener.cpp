// PANDA CONTROLLER
#include "colors.hpp"      //ROS_STRONG_INFO
#include "exceptions.hpp"  //PCEXC
#include "panda.hpp"
#include "panda_controller/teleop_panda.h"  //MSG

// ROS
#include <geometry_msgs/Pose.h>
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
void teleopCallback(const panda_controller::teleop_panda::ConstPtr &msg) {
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "MOVING:");
    const double X = msg->x;
    const double Y = msg->y;
    const double Z = msg->z;
    const double ROLL = msg->roll;
    const double PITCH = msg->pitch;
    const double YAW = msg->yaw;
    ROS_INFO_STREAM("Offset:\n"
                    << "- X:     " << X << std::endl
                    << "- Y:     " << Y << std::endl
                    << "- Z:     " << Z << std::endl
                    << "- ROLL:  " << ROLL << std::endl
                    << "- PITCH: " << PITCH << std::endl
                    << "- YAW:   " << YAW);
    try {
        auto target_pose = panda_ptr->getCurrentPose();
        target_pose.position.x += X;
        target_pose.position.y += Y;
        target_pose.position.z += Z;
        tf2::Quaternion original, orientation, rotation;
        tf2::convert(target_pose.orientation,
                     original);  // get original orientation
        rotation.setRPY(ROLL * M_PI / 180.0, PITCH * M_PI / 180.0,
                        YAW * M_PI / 180.0);  // get rotation orientation
        orientation = rotation * original;   // get new orientation
        orientation.normalize();             // normalize new orientation
        target_pose.orientation = tf2::toMsg(orientation);  // Update

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
        panda_ptr = new robot::Panda();

        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "FIRST POSE:");
        ROS_INFO_STREAM(std::endl << panda_ptr->getCurrentPose());

    } catch (const PCEXC::panda_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Create subscriber
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SUBSCRIPTION TO THE TOPIC: teleop");
    ros::Subscriber sub =
        node.subscribe("/panda_controller/teleop", 1000, teleopCallback);


    // Finish
    ros::waitForShutdown();
    delete panda_ptr;
    return 0;
}