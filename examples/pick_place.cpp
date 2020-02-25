// PANDA CONTROLLER
#include "colors.hpp"  //ROS_STRONG_INFO
#include "data_manager.hpp"
#include "exceptions.hpp"  //PCEXC
#include "panda.hpp"

// ROS
#include <ros/ros.h>



//#############################################################################
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


    // Extract the parameters
    bool GRIPPER_IS_ACTIVE;
    float ARM_SPEED, GRIPPER_SPEED;
    double GRASP_WIDTH, GRASP_FORCE, GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER;
    std::string OBJECT_NAME, SCENE_NAME, PICK_POSE_NAME, PLACE_POSE_NAME;
    if (!(node.getParam("gripper_is_active", GRIPPER_IS_ACTIVE) &&
          node.getParam("scene", SCENE_NAME) &&
          node.getParam("object", OBJECT_NAME) &&
          node.getParam("pick_pose", PICK_POSE_NAME) &&
          node.getParam("place_pose", PLACE_POSE_NAME) &&
          node.getParam("arm_speed", ARM_SPEED) &&
          node.getParam("gripper_speed", GRIPPER_SPEED) &&
          node.getParam("grasp_width", GRASP_WIDTH) &&
          node.getParam("grasp_force", GRASP_FORCE) &&
          node.getParam("grasp_epsilon_inner", GRASP_EPSILON_INNER) &&
          node.getParam("grasp_epsilon_outer", GRASP_EPSILON_OUTER))) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "PANDA CONTROLLER INITIALIZATION");
        auto panda = robot::Panda(GRIPPER_IS_ACTIVE);

        // Init scene
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SCENE INITIALIZATION");
        panda.setScene(data_manager::get_scene("base"));
        panda.setScene(data_manager::get_scene(SCENE_NAME));
        ros::WallDuration(1.0).sleep();

        // Set robot speeds
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SPEEDS ADJUSTAMENT");
        ROS_INFO_STREAM("Arm speed setted to: " << ARM_SPEED);
        panda.setArmSpeed(ARM_SPEED);
        ROS_INFO_STREAM("Gripper speed setted to: " << GRIPPER_SPEED);
        panda.setGripperSpeed(GRIPPER_SPEED);

        // Perform gripper homing
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "GRIPPER HOMING");
        panda.gripperHoming();

        // Pick
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PICKING");
        ROS_INFO_STREAM("Object name: " << OBJECT_NAME);
        ROS_INFO_STREAM("Pick-Pose object: " << PICK_POSE_NAME);
        panda.pick(data_manager::get_pose(PICK_POSE_NAME), OBJECT_NAME,
                   GRASP_WIDTH, GRASP_FORCE, GRASP_EPSILON_INNER,
                   GRASP_EPSILON_OUTER);
        ros::WallDuration(1.0).sleep();

        // Place
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PLACING");
        ROS_INFO_STREAM("Place-Pose object: " << PLACE_POSE_NAME);
        panda.place(data_manager::get_pose(PLACE_POSE_NAME));
        ros::WallDuration(1.0).sleep();

    } catch (const PCEXC::PandaControllerException &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}