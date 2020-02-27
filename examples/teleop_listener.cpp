// PANDA CONTROLLER
#include "colors.hpp"      //ROS_STRONG_INFO
#include "exceptions.hpp"  //PCEXC
#include "panda.hpp"
#include "panda_controller/teleop_panda.h"  //MSG

// ROS
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>



//#############################################################################
// CONFIGS ####################################################################
namespace config {
const auto FG = Colors::FG_BLUE;
const auto BG = Colors::BG_BLACK;


bool GRIPPER_IS_ACTIVE;
float ARM_SPEED, GRIPPER_SPEED, GRIPPER_FORCE, GRIPPER_EPSILON_INNER,
    GRIPPER_EPSILON_OUTER, CARTESIAN_STEP, CARTESIAN_JUMP;
}  // namespace config



//#############################################################################
// GLOBAL VALUES ##############################################################
std::string NAME;
robot::Panda *panda_ptr;



//#############################################################################
// CALLBACK ###################################################################
namespace callback {
double current_gripper_width = robot::config::GRIPPER_MAX_WIDTH;

void teleopCallback(const panda_controller::teleop_panda::ConstPtr &msg) {
    ROS_STRONG_INFO(config::FG, config::BG, "MOVING:");
    const double &X = msg->x;
    const double &Y = msg->y;
    const double &Z = msg->z;
    const double &ROLL = msg->roll;
    const double &PITCH = msg->pitch;
    const double &YAW = msg->yaw;
    const double &ARM_HOMING = msg->arm_homing;
    const bool &GRIPPER_HOMING = msg->gripper_homing;
    const double &GRIPPER_GRASP = msg->gripper_grasp;
    const double &GRIPPER_WIDTH = msg->gripper_width;
    ROS_INFO_STREAM("Offset:\n"
                    << "- X:                " << X << std::endl
                    << "- Y:                " << Y << std::endl
                    << "- Z:                " << Z << std::endl
                    << "- ROLL:             " << ROLL << std::endl
                    << "- PITCH:            " << PITCH << std::endl
                    << "- YAW:              " << YAW << std::endl
                    << "- ARM HOMING:       " << ARM_HOMING << std::endl
                    << "- GRIPPER HOMING:   " << GRIPPER_HOMING << std::endl
                    << "- GRIPPER GRASP:    " << GRIPPER_GRASP << std::endl
                    << "- GRIPPER WITDH:    " << GRIPPER_WIDTH);
    try {
        // GRIPPER HOMING
        if (GRIPPER_HOMING) {
            ROS_INFO("Gripper homing");
            panda_ptr->gripperHoming();
            current_gripper_width = robot::config::GRIPPER_MAX_WIDTH;

            // GRIPPER GRASP
        } else if (GRIPPER_GRASP != 0) {
            if (GRIPPER_GRASP < 0.0 ||
                GRIPPER_GRASP > robot::config::GRIPPER_MAX_WIDTH) {
                ROS_WARN_STREAM("Gripper grasp invalid: " << GRIPPER_GRASP);
            } else {
                ROS_INFO_STREAM("Grasp gripper to: " << GRIPPER_GRASP);
                panda_ptr->gripperGrasp(GRIPPER_GRASP, config::GRIPPER_FORCE,
                                        config::GRIPPER_EPSILON_INNER,
                                        config::GRIPPER_EPSILON_OUTER);
                current_gripper_width = GRIPPER_GRASP;
            }

            // GRIPPER WIDTH
        } else if (GRIPPER_WIDTH != 0) {
            current_gripper_width += GRIPPER_WIDTH;
            if (current_gripper_width < 0.0 ||
                current_gripper_width > robot::config::GRIPPER_MAX_WIDTH) {
                ROS_WARN_STREAM(
                    "Gripper width invalid: " << current_gripper_width);
                current_gripper_width -= GRIPPER_WIDTH;
            } else {
                ROS_INFO_STREAM(
                    "Move gripper to width: " << current_gripper_width);
                panda_ptr->gripperMove(current_gripper_width);
            }

            // ARM HOMING
        } else if (ARM_HOMING) {
            ROS_INFO("Arm homing");
            panda_ptr->moveToReadyPose();

            // ARM
        } else {
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
            panda_ptr->cartesianMovement(target_pose, config::CARTESIAN_STEP,
                                         config::CARTESIAN_JUMP);
        }

    } catch (const PCEXC::PandaControllerException &e) {
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
    ROS_STRONG_INFO(config::FG, config::BG, "START NODE: ", NAME);


    // Extract the parameters
    if (!(node.getParam("gripper_is_active", config::GRIPPER_IS_ACTIVE) &&
          node.getParam("arm_speed", config::ARM_SPEED) &&
          node.getParam("gripper_speed", config::GRIPPER_SPEED) &&
          node.getParam("gripper_force", config::GRIPPER_FORCE) &&
          node.getParam("gripper_epsilon_inner",
                        config::GRIPPER_EPSILON_INNER) &&
          node.getParam("gripper_epsilon_outer",
                        config::GRIPPER_EPSILON_OUTER) &&
          node.getParam("cartesian_step", config::CARTESIAN_STEP) &&
          node.getParam("cartesian_jump", config::CARTESIAN_JUMP))) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_STRONG_INFO(config::FG, config::BG,
                        "PANDA CONTROLLER INITIALIZATION");
        panda_ptr = new robot::Panda(config::GRIPPER_IS_ACTIVE);

        // Set end effector link
        ROS_STRONG_INFO(config::FG, config::BG, "EEF SETTING:");
        ROS_INFO_STREAM(
            "End Effector link setted to: " << robot::config::CENTER_EEF);
        panda_ptr->setEndEffectorLink(robot::config::CENTER_EEF);

        // Set robot
        ROS_STRONG_INFO(config::FG, config::BG, "SPEEDS ADJUSTAMENT");
        ROS_INFO_STREAM("Arm speed setted to: " << config::ARM_SPEED);
        panda_ptr->setArmSpeed(config::ARM_SPEED);
        ROS_INFO_STREAM("Gripper speed setted to: " << config::GRIPPER_SPEED);
        panda_ptr->setGripperSpeed(config::GRIPPER_SPEED);
        ROS_INFO("Gripper homing");
        panda_ptr->gripperHoming();

        // Print start pose
        ROS_STRONG_INFO(config::FG, config::BG, "START POSE:");
        ROS_INFO_STREAM(std::endl << panda_ptr->getCurrentPose());

    } catch (const PCEXC::PandaControllerException &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Create subscriber
    ROS_STRONG_INFO(config::FG, config::BG,
                    "SUBSCRIPTION TO THE TOPIC: teleop");
    ros::Subscriber sub = node.subscribe("/panda_controller/teleop", 1000,
                                         callback::teleopCallback);


    // Finish
    ros::waitForShutdown();
    delete panda_ptr;
    return 0;
}