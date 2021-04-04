// ROS
#include <ros/ros.h>

// C++
#include <iostream>

// Custom
#include "panda_controller/exceptions.hpp"
#include "panda_controller/panda.hpp"
#include "utils/colors.hpp"



// USING NAMESPACE ============================================================
using namespace panda_controller;



// CONFIGS ====================================================================
auto FG_COLOR = Colors::FG_BLUE;
auto BG_COLOR = Colors::BG_DEFAULT;
auto INFO_COLOR = Colors::FG_CYAN;



// MAIN =======================================================================
int main(int argc, char **argv) {
    // Get the file name
    std::string file_path = argv[0];
    const std::string CURRENT_FILE_NAME =
        file_path.substr(file_path.find_last_of("/\\") + 1);


    // Setup ROS
    ros::init(argc, argv, CURRENT_FILE_NAME);
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "START NODE: ", CURRENT_FILE_NAME);


    // Extract the parameters
    bool REAL_ROBOT, GRIPPER_HOMING;
    double ARM_VELOCITY_FACTOR;
    double GRASP_WIDTH, GRASP_SPEED, GRASP_FORCE, GRASP_EPSILON_INNER,
        GRASP_EPSILON_OUTER;
    double GRIPPER_SPEED;
    double EEF_STEP, JUMP_THRESHOLD;
    std::string ENV_SCENE, TASK_SCENE;
    std::string PRE_GRASP_POSE_NAME, OBJECT_POSE_NAME;
    std::string POST_GRASP_POSE_NAME, GOAL_POSE_NAME, POST_PLACE_POSE_NAME;
    if (!(node.getParam("arm_velocity_factor", ARM_VELOCITY_FACTOR) &&
          node.getParam("real_robot", REAL_ROBOT) &&
          node.getParam("gripper_homing", GRIPPER_HOMING) &&
          node.getParam("gripper_speed", GRIPPER_SPEED) &&
          node.getParam("grasp_width", GRASP_WIDTH) &&
          node.getParam("grasp_speed", GRASP_SPEED) &&
          node.getParam("grasp_force", GRASP_FORCE) &&
          node.getParam("grasp_epsilon_inner", GRASP_EPSILON_INNER) &&
          node.getParam("grasp_epsilon_outer", GRASP_EPSILON_OUTER) &&
          node.getParam("eef_step", EEF_STEP) &&
          node.getParam("jump_threshold", JUMP_THRESHOLD) &&
          node.getParam("env_scene", ENV_SCENE) &&
          node.getParam("task_scene", TASK_SCENE) &&
          node.getParam("pre_grasp_pose", PRE_GRASP_POSE_NAME) &&
          node.getParam("object_pose", OBJECT_POSE_NAME) &&
          node.getParam("post_grasp_pose", POST_GRASP_POSE_NAME) &&
          node.getParam("goal_pose", GOAL_POSE_NAME) &&
          node.getParam("post_place_pose", POST_PLACE_POSE_NAME))) {
        ROS_FATAL_STREAM(
            get_err_msg(CURRENT_FILE_NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Init panda
        ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "PANDA CONTROLLER INITIALIZATION");
        auto panda = Panda(REAL_ROBOT, 1.0);
        auto arm = panda.getArm();
        auto gripper = panda.getGripper();
        auto scene = panda.getScene();
        arm->setMaxVelocityScalingFactor(ARM_VELOCITY_FACTOR);
        if (GRIPPER_HOMING)
            gripper->homing();


        // Init scene
        ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "SCENE INITIALIZATION");
        scene->setScene(ENV_SCENE);
        scene->setScene(TASK_SCENE);


        // Pick
        ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "OBJECT PICKING");
        ROS_COL_INFO(INFO_COLOR, "Pre-Grasp pose: ", PRE_GRASP_POSE_NAME);
        auto pre_grasp_pose = arm->getPose(PRE_GRASP_POSE_NAME);
        ROS_INFO_STREAM("pose:\n" << pre_grasp_pose);

        ROS_COL_INFO(INFO_COLOR, "Object pose: ", OBJECT_POSE_NAME);
        auto object_pose = arm->getPose(OBJECT_POSE_NAME);
        ROS_INFO_STREAM("pose:\n" << object_pose);

        panda.pick(pre_grasp_pose, object_pose, GRASP_WIDTH, GRASP_FORCE,
                   GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER, EEF_STEP,
                   JUMP_THRESHOLD);
        ros::WallDuration(1.0).sleep();


        // Place
        ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "OBJECT PLACING");
        ROS_COL_INFO(INFO_COLOR, "Post-Grasp pose: ", POST_GRASP_POSE_NAME);
        auto post_grasp = arm->getPose(POST_GRASP_POSE_NAME);
        ROS_INFO_STREAM("pose:\n" << post_grasp);

        ROS_COL_INFO(INFO_COLOR, "Goal pose: ", GOAL_POSE_NAME);
        auto goal_pose = arm->getPose(GOAL_POSE_NAME);
        ROS_INFO_STREAM("pose:\n" << goal_pose);

        ROS_COL_INFO(INFO_COLOR, "Post-place pose: ", POST_PLACE_POSE_NAME);
        auto post_place_pose = arm->getPose(POST_PLACE_POSE_NAME);
        ROS_INFO_STREAM("pose:\n" << post_place_pose);

        panda.place(post_grasp, goal_pose, post_place_pose, EEF_STEP,
                    JUMP_THRESHOLD);


    } catch (const PandaControllerErr &err) {
        ROS_FATAL_STREAM(get_err_msg(CURRENT_FILE_NAME, err.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}
