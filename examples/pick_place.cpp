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
    int METHOD;
    float ARM_SPEED, GRIPPER_SPEED;
    double GRASP_WIDTH, GRASP_FORCE, GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER;
    double PRE_GRASP_APPROCH_X, PRE_GRASP_APPROCH_Y, PRE_GRASP_APPROCH_Z;
    double POST_GRASP_RETREAT_X, POST_GRASP_RETREAT_Y, POST_GRASP_RETREAT_Z;
    double POST_PLACE_RETREAT_X, POST_PLACE_RETREAT_Y, POST_PLACE_RETREAT_Z;
    std::string OBJECT_NAME, SCENE_NAME, PICK_POSE_NAME, PLACE_POSE_NAME;
    if (!(node.getParam("method", METHOD) &&
          node.getParam("scene", SCENE_NAME) &&
          node.getParam("object", OBJECT_NAME) &&
          node.getParam("pick_pose", PICK_POSE_NAME) &&
          node.getParam("place_pose", PLACE_POSE_NAME) &&
          node.getParam("arm_speed", ARM_SPEED) &&
          node.getParam("gripper_speed", GRIPPER_SPEED) &&
          node.getParam("grasp_width", GRASP_WIDTH) &&
          node.getParam("grasp_force", GRASP_FORCE) &&
          node.getParam("grasp_epsilon_inner", GRASP_EPSILON_INNER) &&
          node.getParam("grasp_epsilon_outer", GRASP_EPSILON_OUTER) &&
          node.getParam("pre_grasp_approch_x", PRE_GRASP_APPROCH_X) &&
          node.getParam("pre_grasp_approch_y", PRE_GRASP_APPROCH_Y) &&
          node.getParam("pre_grasp_approch_z", PRE_GRASP_APPROCH_Z) &&
          node.getParam("post_grasp_retreat_x", POST_GRASP_RETREAT_X) &&
          node.getParam("post_grasp_retreat_y", POST_GRASP_RETREAT_Y) &&
          node.getParam("post_grasp_retreat_z", POST_GRASP_RETREAT_Z) &&
          node.getParam("post_place_retreat_x", POST_PLACE_RETREAT_X) &&
          node.getParam("post_place_retreat_y", POST_PLACE_RETREAT_Y) &&
          node.getParam("post_place_retreat_z", POST_PLACE_RETREAT_Z))) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "PANDA CONTROLLER INITIALIZATION");
        auto panda = robot::Panda();

        // Init scene
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SCENE INITIALIZATION");
        panda.setScene(data_manager::get_scene(SCENE_NAME));
        ros::WallDuration(1.0).sleep();

        // Set robot speeds
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SPEEDS ADJUSTAMENT");
        ROS_INFO_STREAM("Arm speed setted to: " << ARM_SPEED);
        panda.setArmSpeed(ARM_SPEED);
        ROS_INFO_STREAM("Gripper speed setted to: " << GRIPPER_SPEED);
        panda.setArmSpeed(GRIPPER_SPEED);

        // Perform gripper homing
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "GRIPPER HOMING");
        panda.gripperHoming();  // needs real robot


        // Pick and place
        if (METHOD == 1) {  // *************************************************
            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED METHOD1: only pose");
            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "PICKING OBJECT");
            ROS_INFO_STREAM("Pose object: " << PICK_POSE_NAME);
            panda.pick(data_manager::get_pose(PICK_POSE_NAME), GRASP_WIDTH,
                       GRASP_FORCE, GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER);
            ros::WallDuration(1.0).sleep();

            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PLACING");
            ROS_INFO_STREAM("Place-Pose object: " << PLACE_POSE_NAME);
            panda.place(data_manager::get_pose(PLACE_POSE_NAME));
            ros::WallDuration(1.0).sleep();

        } else if (METHOD == 2) {  // ******************************************
            ROS_STRONG_INFO(FG_COLOR, BG_COLOR,
                            "SELECTED METHOD2: pose with approches");
            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PICKING");
            ROS_INFO_STREAM("Pick-Pose object: " << PICK_POSE_NAME);
            geometry_msgs::Vector3 PRE_GRASP_APPROCH;
            PRE_GRASP_APPROCH.x = PRE_GRASP_APPROCH_X;
            PRE_GRASP_APPROCH.y = PRE_GRASP_APPROCH_Y;
            PRE_GRASP_APPROCH.z = PRE_GRASP_APPROCH_Z;
            panda.pick(data_manager::get_pose(PICK_POSE_NAME),
                       PRE_GRASP_APPROCH, GRASP_WIDTH, GRASP_FORCE,
                       GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER);
            ros::WallDuration(1.0).sleep();

            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PLACING");
            ROS_INFO_STREAM("Place-Pose object: " << PLACE_POSE_NAME);
            geometry_msgs::Vector3 POST_GRASP_RETREAT;
            POST_GRASP_RETREAT.x = POST_GRASP_RETREAT_X;
            POST_GRASP_RETREAT.y = POST_GRASP_RETREAT_Y;
            POST_GRASP_RETREAT.z = POST_GRASP_RETREAT_Z;
            geometry_msgs::Vector3 POST_PLACE_RETREAT;
            POST_PLACE_RETREAT.x = POST_PLACE_RETREAT_X;
            POST_PLACE_RETREAT.y = POST_PLACE_RETREAT_Y;
            POST_PLACE_RETREAT.z = POST_PLACE_RETREAT_Z;
            panda.place(data_manager::get_pose(PLACE_POSE_NAME),
                        POST_GRASP_RETREAT, POST_PLACE_RETREAT);
            ros::WallDuration(1.0).sleep();

        } else if (METHOD == 3) {  // ******************************************
            ROS_STRONG_INFO(FG_COLOR, BG_COLOR,
                            "SELECTED METHOD3: pose and collision object");
            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PICKING");
            ROS_INFO_STREAM("Object name: " << OBJECT_NAME);
            ROS_INFO_STREAM("Pick-Pose object: " << PICK_POSE_NAME);
            panda.pick(data_manager::get_pose(PICK_POSE_NAME), OBJECT_NAME,
                       GRASP_FORCE, GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER);
            ros::WallDuration(1.0).sleep();

            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PLACING");
            ROS_INFO_STREAM("Place-Pose object: " << PLACE_POSE_NAME);
            panda.place(data_manager::get_pose(PLACE_POSE_NAME));
            ros::WallDuration(1.0).sleep();

        } else {
            ROS_FATAL_STREAM("METHOD SELECTED WRONG");
        }

    } catch (const PCEXC::panda_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    } catch (const PCEXC::data_manager_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}