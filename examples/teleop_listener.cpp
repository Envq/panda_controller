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
namespace callback {
double current_gripper_width = 0.0;

void teleopCallback(const panda_controller::teleop_panda::ConstPtr &msg) {
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "MOVING:");
    const double X = msg->x;
    const double Y = msg->y;
    const double Z = msg->z;
    const double ROLL = msg->roll;
    const double PITCH = msg->pitch;
    const double YAW = msg->yaw;
    const double GRIPPER_WIDTH = msg->gripper_width;
    const bool GRIPPER_HOMING = msg->gripper_homing;
    ROS_INFO_STREAM("Offset:\n"
                    << "- X:                " << X << std::endl
                    << "- Y:                " << Y << std::endl
                    << "- Z:                " << Z << std::endl
                    << "- ROLL:             " << ROLL << std::endl
                    << "- PITCH:            " << PITCH << std::endl
                    << "- YAW:              " << YAW << std::endl
                    << "- GRIPPER HOMING:   " << GRIPPER_HOMING << std::endl
                    << "- GRIPPER WITDH:    " << GRIPPER_WIDTH);
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
        orientation = rotation * original;    // get new orientation
        orientation.normalize();              // normalize new orientation
        target_pose.orientation = tf2::toMsg(orientation);  // Update

        ROS_INFO_STREAM("Move to pose:\n" << target_pose);
        panda_ptr->cartesianMovement(target_pose);

        if (GRIPPER_WIDTH != 0.0) {
            current_gripper_width += GRIPPER_WIDTH;
            ROS_INFO_STREAM("Move gripper to width: " << current_gripper_width);
            panda_ptr->gripperMove(current_gripper_width);
        }

        if (GRIPPER_WIDTH == 0.0 && GRIPPER_HOMING) {
            panda_ptr->gripperHoming();
        }

    } catch (const PCEXC::panda_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }
}
}  // namespace callback



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


    // Extract the parameters
    float ARM_SPEED, GRIPPER_SPEED;
    if (!(node.getParam("arm_speed", ARM_SPEED) &&
          node.getParam("gripper_speed", GRIPPER_SPEED))) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "PANDA CONTROLLER INITIALIZATION");
        panda_ptr = new robot::Panda();

        // Set robot speeds
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SPEEDS ADJUSTAMENT");
        ROS_INFO_STREAM("Arm speed setted to: " << ARM_SPEED);
        panda_ptr->setArmSpeed(ARM_SPEED);
        ROS_INFO_STREAM("Gripper speed setted to: " << GRIPPER_SPEED);
        panda_ptr->setArmSpeed(GRIPPER_SPEED);

        // Print first pose
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "FIRST POSE:");
        ROS_INFO_STREAM(std::endl << panda_ptr->getCurrentPose());

    } catch (const PCEXC::panda_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Create subscriber
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SUBSCRIPTION TO THE TOPIC: teleop");
    ros::Subscriber sub = node.subscribe("/panda_controller/teleop", 1000,
                                         callback::teleopCallback);


    // Finish
    ros::waitForShutdown();
    delete panda_ptr;
    return 0;
}