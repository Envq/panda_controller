// ROS
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// Custom
#include "panda_controller/exceptions.hpp"
#include "panda_controller/panda_arm.hpp"
#include "panda_controller/panda_gripper.hpp"
#include "panda_controller/panda_teleop.h"  // msg
#include "utils/colors.hpp"

// BOOST
#include <boost/shared_ptr.hpp>



// USING NAMESPACE ============================================================
using namespace panda_controller;



// CONFIGS ====================================================================
auto FG_COLOR = Colors::FG_BLUE;
auto BG_COLOR = Colors::BG_DEFAULT;
auto INFO_COLOR = Colors::FG_GREEN;



// GLOBAL VARS ================================================================
std::string CURRENT_FILE_NAME;
std::shared_ptr<PandaArm> arm_ptr;
std::shared_ptr<PandaGripper> gripper_ptr;
bool REAL_ROBOT, GR_HOMING;
float ARM_VELOCITY_FACTOR, GRIPPER_SPEED, GRASP_SPEED, GRASP_FORCE,
    GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER, EEF_STEP, JUMP_THRESHOLD;



// CALLBACK ===================================================================
void teleopCallback(const panda_controller::panda_teleop::ConstPtr &msg) {
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "MOVING:\n");
    const double &X = msg->x;
    const double &Y = msg->y;
    const double &Z = msg->z;
    const double &ROLL = msg->roll;
    const double &PITCH = msg->pitch;
    const double &YAW = msg->yaw;
    const double &READY = msg->arm_homing;
    const bool &GR_HOMING = msg->gripper_homing;
    const double &GR_GRASP = msg->gripper_grasp;
    const double &GR_WIDTH = msg->gripper_width;
    ROS_INFO_STREAM("Message received:\n"
                    << "- X:                " << X << std::endl
                    << "- Y:                " << Y << std::endl
                    << "- Z:                " << Z << std::endl
                    << "- ROLL:             " << ROLL << std::endl
                    << "- PITCH:            " << PITCH << std::endl
                    << "- YAW:              " << YAW << std::endl
                    << "- READY:            " << READY << std::endl
                    << "- GRIPPER HOMING:   " << GR_HOMING << std::endl
                    << "- GRIPPER GRASP:    " << GR_GRASP << std::endl
                    << "- GRIPPER WITDH:    " << GR_WIDTH);

    try {
        if (GR_HOMING != 0) {
            ROS_INFO("Gripper Homing");
            gripper_ptr->homing();
            throw PandaGripperErr("test");


        } else if (GR_GRASP != 0) {
            ROS_INFO("Gripper Grasp");
            ROS_INFO_STREAM("Grasp gripper to: " << GR_GRASP);
            gripper_ptr->grasp(GR_GRASP, GRASP_FORCE, GRASP_EPSILON_INNER,
                               GRASP_EPSILON_OUTER);


        } else if (GR_WIDTH != 0) {
            ROS_INFO("Gripper Move");
            double target_width = gripper_ptr->getWidth() + GR_WIDTH;
            gripper_ptr->move(target_width);


        } else if (READY) {
            ROS_INFO("Arm to Ready");
            arm_ptr->moveToReady();


        } else {
            ROS_INFO("Relative Move");
            arm_ptr->relativeMove(X, Y, Z, ROLL, PITCH, YAW);
        }

    } catch (const PandaControllerErr &err) {
        ROS_FATAL_STREAM(get_err_msg(CURRENT_FILE_NAME, err.what()));
        ros::shutdown();
    }
}



// MAIN
// =======================================================================
int main(int argc, char **argv) {
    // Get the file name
    std::string file_path = argv[0];
    CURRENT_FILE_NAME = file_path.substr(file_path.find_last_of("/\\") + 1);


    // Setup ROS
    ros::init(argc, argv, CURRENT_FILE_NAME);
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "START NODE: ", CURRENT_FILE_NAME);


    // Extract the parameters
    if (!(node.getParam("arm_velocity_factor", ARM_VELOCITY_FACTOR) &&
          node.getParam("real_robot", REAL_ROBOT) &&
          node.getParam("gripper_homing", GR_HOMING) &&
          node.getParam("gripper_speed", GRIPPER_SPEED) &&
          node.getParam("grasp_speed", GRASP_SPEED) &&
          node.getParam("grasp_force", GRASP_FORCE) &&
          node.getParam("grasp_epsilon_inner", GRASP_EPSILON_INNER) &&
          node.getParam("grasp_epsilon_outer", GRASP_EPSILON_OUTER) &&
          node.getParam("eef_step", EEF_STEP) &&
          node.getParam("jump_threshold", JUMP_THRESHOLD))) {
        ROS_FATAL_STREAM(get_err_msg(CURRENT_FILE_NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "PANDA CONTROLLER INITIALIZATION");
        arm_ptr.reset(new PandaArm(1.0));
        gripper_ptr.reset(new PandaGripper(REAL_ROBOT));
        arm_ptr->setMaxVelocityScalingFactor(ARM_VELOCITY_FACTOR);
        if (GR_HOMING)
            gripper_ptr->homing();


        // Print start pose
        ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "START POSE:");
        ROS_INFO_STREAM("Pose\n" << arm_ptr->getPose());

    } catch (const PandaControllerErr &err) {
        ROS_FATAL_STREAM(get_err_msg(CURRENT_FILE_NAME, err.what()));
    }


    // Create subscriber
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "SUBSCRIPTION TO THE TOPIC: teleoperation");
    ros::Subscriber sub =
        node.subscribe("/panda_controller/teleoperation", 1000, teleopCallback);


    // Finish
    ros::waitForShutdown();
    return 0;
}