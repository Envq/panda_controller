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
    float SPEED;
    std::string OBJECT_NAME, SCENE_NAME, PICK_POSE_NAME, PLACE_POSE_NAME;
    if (!(node.getParam("speed", SPEED) && node.getParam("scene", SCENE_NAME) &&
          node.getParam("object", OBJECT_NAME) &&
          node.getParam("pick_pose", PICK_POSE_NAME) &&
          node.getParam("place_pose", PLACE_POSE_NAME))) {
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

        // Set robot speed
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "ARM SPEED ADJUSTAMENT");
        ROS_INFO_STREAM("Speed setted to: " << SPEED);
        panda.setArmSpeed(SPEED);

        // Set robot speed
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "GRIPPER HOMING");
        // panda.gripperHoming();


        // Pick and place
        // METHOD1 ************************************************************
        // ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "PICKING OBJECT");
        // ROS_INFO_STREAM("Pose object: " << PICK_POSE_NAME);
        // panda.pick(data_manager::get_pose(PICK_POSE_NAME), 0.02);
        // ros::WallDuration(1.0).sleep();

        // ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PLACING");
        // ROS_INFO_STREAM("Place-Pose object: " << PLACE_POSE_NAME);
        // panda.place(data_manager::get_pose(PLACE_POSE_NAME));
        // ros::WallDuration(1.0).sleep();


        // METHOD2 ************************************************************
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PICKING");
        ROS_INFO_STREAM("Pick-Pose object: " << PICK_POSE_NAME);
        geometry_msgs::Vector3 PRE_GRASP_APPROCH;
        PRE_GRASP_APPROCH.x = 0.0;
        PRE_GRASP_APPROCH.y = 0.0;
        PRE_GRASP_APPROCH.z = 0.1;
        panda.pick(data_manager::get_pose(PICK_POSE_NAME), 0.02,
                   PRE_GRASP_APPROCH);
        ros::WallDuration(1.0).sleep();

        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "OBJECT PLACING");
        ROS_INFO_STREAM("Place-Pose object: " << PLACE_POSE_NAME);
        geometry_msgs::Vector3 POST_GRASP_RETREAT;
        POST_GRASP_RETREAT.x = 0.0;
        POST_GRASP_RETREAT.y = 0.0;
        POST_GRASP_RETREAT.z = 0.1;
        geometry_msgs::Vector3 POST_PLACE_RETREAT;
        POST_PLACE_RETREAT.x = -0.1;
        POST_PLACE_RETREAT.y = 0.0;
        POST_PLACE_RETREAT.z = 0.0;
        panda.place(data_manager::get_pose(PLACE_POSE_NAME), POST_GRASP_RETREAT,
                    POST_PLACE_RETREAT);
        ros::WallDuration(1.0).sleep();


    } catch (const PCEXC::panda_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));

    } catch (const PCEXC::data_manager_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}