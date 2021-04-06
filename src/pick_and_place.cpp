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


    // Creat pick_and_place structure
    struct pick_and_place {
        std::string pre_grasp;
        std::string object;
        std::string post_grasp;

        std::string pre_place;
        std::string goal;
        std::string post_place;
    };
    size_t SIZE = 2;  // MODIFY THIS: change the number of tasks here.
    pick_and_place TASKS[SIZE];


    // Extract the parameters
    bool REAL_ROBOT, GRIPPER_HOMING;
    double ARM_VELOCITY_FACTOR;
    double GRASP_WIDTH, GRASP_SPEED, GRASP_FORCE, GRASP_EPSILON_INNER,
        GRASP_EPSILON_OUTER;
    double GRIPPER_SPEED;
    double EEF_STEP, JUMP_THRESHOLD;
    std::string ENV_SCENE, TASK_SCENE;
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

          node.getParam("pre_grasp_name_1", TASKS[0].pre_grasp) &&
          node.getParam("object_name_1", TASKS[0].object) &&
          node.getParam("post_grasp_name_1", TASKS[0].post_grasp) &&
          node.getParam("pre_place_name_1", TASKS[0].pre_place) &&
          node.getParam("goal_name_1", TASKS[0].goal) &&
          node.getParam("post_place_name_1", TASKS[0].post_place) &&

          node.getParam("pre_grasp_name_2", TASKS[1].pre_grasp) &&
          node.getParam("object_name_2", TASKS[1].object) &&
          node.getParam("post_grasp_name_2", TASKS[1].post_grasp) &&
          node.getParam("pre_place_name_2", TASKS[1].pre_place) &&
          node.getParam("goal_name_2", TASKS[1].goal) &&
          node.getParam("post_place_name_2", TASKS[1].post_place) &&

          // MODIFY THIS: add other task here.

          node.getParam("env_scene", ENV_SCENE) &&
          node.getParam("task_scene", TASK_SCENE))) {
        ROS_FATAL_STREAM(get_err_msg(CURRENT_FILE_NAME, "Can't get parameters"));
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
        scene->resetScene();
        scene->setScene(ENV_SCENE);
        scene->setScene(TASK_SCENE);


        // Perform tasks:
        for (size_t i = 0; i < SIZE; i++) {
            auto &task = TASKS[i];
            ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "TASK: ", SIZE);

            // Pick
            ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "OBJECT PICKING");

            ROS_COL_INFO(INFO_COLOR, "Pre-Grasp pose: ", task.pre_grasp);
            auto pre_grasp_pose = arm->getPose(task.pre_grasp);
            ROS_INFO_STREAM("pose:\n" << pre_grasp_pose);

            ROS_COL_INFO(INFO_COLOR, "Object pose: ", task.object);
            auto object_pose = arm->getPose(task.object);
            ROS_INFO_STREAM("pose:\n" << object_pose);

            ROS_COL_INFO(INFO_COLOR, "Post-Grasp pose: ", task.post_grasp);
            auto post_grasp_pose = arm->getPose(task.post_grasp);
            ROS_INFO_STREAM("pose:\n" << post_grasp_pose);

            panda.pick(pre_grasp_pose, object_pose, post_grasp_pose, GRASP_WIDTH,
                       GRASP_FORCE, GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER,
                       EEF_STEP, JUMP_THRESHOLD);
            ros::WallDuration(1.0).sleep();


            // Place
            ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "OBJECT PLACING");

            ROS_COL_INFO(INFO_COLOR, "Pre-Place pose: ", task.pre_place);
            auto pre_place_pose = arm->getPose(task.pre_place);
            ROS_INFO_STREAM("pose:\n" << pre_place_pose);

            ROS_COL_INFO(INFO_COLOR, "Goal pose: ", task.goal);
            auto goal_pose = arm->getPose(task.goal);
            ROS_INFO_STREAM("pose:\n" << goal_pose);

            ROS_COL_INFO(INFO_COLOR, "Post-place pose: ", task.post_place);
            auto post_place_pose = arm->getPose(task.post_place);
            ROS_INFO_STREAM("pose:\n" << post_place_pose);

            panda.place(pre_place_pose, goal_pose, post_place_pose, EEF_STEP,
                        JUMP_THRESHOLD);

            std::cout << "--------------------------------------" << std::endl;
            ros::WallDuration(1.0).sleep();
        }
    } catch (const PandaControllerErr &err) {
        ROS_FATAL_STREAM(get_err_msg(CURRENT_FILE_NAME, err.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}
