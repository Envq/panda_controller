// MY LIBS
#include "arm.hpp"
#include "data_manager.hpp"
#include "my_exceptions.hpp"

// ROS
#include <ros/ros.h>



//#############################################################################
// PARAMETERS



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
    std::string OBJECT_NAME, SCENE_NAME, PICK_POSE_NAME, PLACE_POSE_NAME;
    if (!node.getParam("speed", SPEED) || !node.getParam("scene", SCENE_NAME) ||
        !node.getParam("object", OBJECT_NAME) ||
        !node.getParam("pick_pose", PICK_POSE_NAME) ||
        !node.getParam("place_pose", PLACE_POSE_NAME)) {
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
        ROS_INFO_STREAM(">> SET SPEED: " << SPEED);
        panda.setSpeed(SPEED);

        // Init scene
        ROS_INFO(">> INIT SCENE");
        panda.setScene(data_manager::get_scene(SCENE_NAME));
        ros::WallDuration(1.0).sleep();

        // Pick object
        ROS_INFO(">> PICK OBJECT");
        panda.pick(data_manager::get_pose(PICK_POSE_NAME), OBJECT_NAME);
        ros::WallDuration(1.0).sleep();

        // Place Object
        ROS_INFO(">> PLACE OBJECT");
        panda.place(data_manager::get_pose(PLACE_POSE_NAME), OBJECT_NAME);
        ros::WallDuration(1.0).sleep();


    } catch (const my_exceptions::data_manager_error &e) {
        ROS_FATAL_STREAM(">> " << e.what());

    } catch (const my_exceptions::arm_error &e) {
        ROS_FATAL_STREAM(">> " << e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}