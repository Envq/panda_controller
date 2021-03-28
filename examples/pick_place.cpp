// PANDA CONTROLLER
#include "colors.hpp"  //ROS_STRONG_INFO
#include "data_manager.hpp"
#include "exceptions.hpp"  //PCEXC
#include "panda.hpp"

// ROS
#include <ros/ros.h>



//#############################################################################
// CONFIGS ####################################################################
namespace config {
const auto FG = Colors::FG_BLUE;
const auto BG = Colors::BG_BLACK;
}  // namespace config



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
    ROS_STRONG_INFO(config::FG, config::BG, "START NODE: ", NAME);


    std::string SCENE_NAME;
    struct pick_place {
        std::string object;
        std::string pick;
        std::string place;
    };

    // Extract the parameters
    pick_place TASK[2];
    bool GRIPPER_IS_ACTIVE;
    float ARM_SPEED, GRIPPER_SPEED;
    double GRASP_WIDTH, GRASP_FORCE, GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER;
    if (!(node.getParam("gripper_is_active", GRIPPER_IS_ACTIVE) &&
          node.getParam("scene", SCENE_NAME) &&
          node.getParam("object1", TASK[0].object) &&
          node.getParam("pick_pose1", TASK[0].pick) &&
          node.getParam("place_pose1", TASK[0].place) &&
          node.getParam("object2", TASK[1].object) &&
          node.getParam("pick_pose2", TASK[1].pick) &&
          node.getParam("place_pose2", TASK[1].place) &&
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
        ROS_STRONG_INFO(config::FG, config::BG,
                        "PANDA CONTROLLER INITIALIZATION");
        auto panda = robot::Panda(GRIPPER_IS_ACTIVE);

        // Set end effector link
        ROS_STRONG_INFO(config::FG, config::BG, "EEF SETTING:");
        ROS_INFO_STREAM(
            "End Effector link setted to: " << robot::config::CENTER_EEF);
        panda.setEndEffectorLink(robot::config::CENTER_EEF);

        // Init scene
        ROS_STRONG_INFO(config::FG, config::BG, "SCENE INITIALIZATION");
        panda.setScene(data_manager::get_scene("base"));
        panda.setScene(data_manager::get_scene(SCENE_NAME));
        ros::WallDuration(1.0).sleep();

        // Set robot speeds
        ROS_STRONG_INFO(config::FG, config::BG, "SPEEDS ADJUSTAMENT");
        ROS_INFO_STREAM("Arm speed setted to: " << ARM_SPEED);
        panda.setArmSpeed(ARM_SPEED);
        ROS_INFO_STREAM("Gripper speed setted to: " << GRIPPER_SPEED);
        panda.setGripperSpeed(GRIPPER_SPEED);

        // Perform gripper homing
        ROS_STRONG_INFO(config::FG, config::BG, "GRIPPER HOMING");
        panda.gripperHoming();

        // Perform jobs:
        for (const auto JOB : TASK) {
            // Pick
            ROS_STRONG_INFO(config::FG, config::BG, "OBJECT PICKING");
            ROS_INFO_STREAM("Object name: " << JOB.object);
            ROS_INFO_STREAM("Pick-Pose object: " << JOB.pick);
            panda.pick(data_manager::get_pose(JOB.pick), JOB.object,
                       GRASP_WIDTH, GRASP_FORCE, GRASP_EPSILON_INNER,
                       GRASP_EPSILON_OUTER);
            ros::WallDuration(1.0).sleep();

            // Place
            ROS_STRONG_INFO(config::FG, config::BG, "OBJECT PLACING");
            ROS_INFO_STREAM("Place-Pose object: " << JOB.place);
            panda.place(data_manager::get_pose(JOB.place));
            ros::WallDuration(1.0).sleep();
        }

    } catch (const PCEXC::PandaControllerException &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}