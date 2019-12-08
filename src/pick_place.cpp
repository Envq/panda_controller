// MY LIBS
#include "arm.hpp"
#include "data_manager.hpp"
#include "my_exceptions.hpp"

// ROS
#include <ros/ros.h>



//#############################################################################
// PARAMETERS
const auto &SCENE_NAME = "altair";
const auto &OBJECT_NAME = "object";
const auto &PICK_POSE_NAME = "pick";
const auto &PICK_SURFACE = "surface";
const auto &PLACE_POSE_NAME = "place";
const auto &PLACE_SURFACE = "surface";



//#############################################################################
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
    ROS_INFO_STREAM(">> START: " << NAME);


    // Extract the parameters
    float SPEED;
    if (!node.getParam("speed", SPEED)) {
        ROS_FATAL_STREAM(
            ">> Can't get parameters. (Don't use rosrun. Use roslaunch)!");
        ros::shutdown();
        return 0;
    }



    // Task
    try {
        // Create class to manage the Panda arm
        ROS_INFO(">> INIT PANDA CONTROLLER");
        auto panda = arm::Panda();

        // Set robot speed
        // TODO: FIX -> not work in pick_place
        ROS_INFO_STREAM(">> SET SPEED: " << SPEED);
        panda.setSpeed(SPEED);

        // Init scene
        ROS_INFO(">> INIT SCENE");
        panda.setScene(data_manager::get_scene(SCENE_NAME));
        ros::WallDuration(1.0).sleep();

        // Get current pose
        // ROS_INFO(">> GET CURRENT POSE");
        // auto start_pose = panda.getCurrentPose();
        // ros::WallDuration(1.0).sleep();

        // Pick object
        ROS_INFO(">> PICK OBJECT");
        panda.pick(OBJECT_NAME, PICK_SURFACE, arm::get_vector_with("neg_z"),
                   arm::get_vector_with("pos_z"),
                   data_manager::get_pose(PICK_POSE_NAME));
        ros::WallDuration(1.0).sleep();

        // Place Object
        ROS_INFO(">> PLACE OBJECT");
        panda.place(OBJECT_NAME, PLACE_SURFACE, arm::get_vector_with("neg_z"),
                    arm::get_vector_with("pos_z"),
                    data_manager::get_pose(PLACE_POSE_NAME));
        ros::WallDuration(1.0).sleep();

        // Return to start_pose
        // ROS_INFO(">> RETURN TO START POSE");
        // panda.moveToPosition(start_pose);
        // ros::WallDuration(1.0).sleep();


    } catch (const my_exceptions::data_manager_error &e) {
        ROS_FATAL_STREAM(">> " << e.what());

    } catch (const my_exceptions::arm_error &e) {
        ROS_FATAL_STREAM(">> " << e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}