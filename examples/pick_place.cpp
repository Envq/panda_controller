// MY LIBS
#include "data_manager.hpp"
#include "my_exceptions.hpp"
#include "panda.hpp"

// ROS
#include <ros/ros.h>



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
    ROS_INFO_STREAM("## START: " << NAME);


    // Extract the parameters
    float SPEED;
    std::string OBJECT_NAME, SCENE_NAME, PICK_POSE_NAME, PLACE_POSE_NAME;
    if (!(node.getParam("speed", SPEED) && node.getParam("scene", SCENE_NAME) &&
          node.getParam("object", OBJECT_NAME) &&
          node.getParam("pick_pose", PICK_POSE_NAME) &&
          node.getParam("place_pose", PLACE_POSE_NAME))) {
        ROS_FATAL_STREAM(
            my_exceptions::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_INFO("## INIT PANDA CONTROLLER");
        auto panda = robot::Panda(true);

        // Set robot speed
        ROS_INFO_STREAM("## SET SPEED: " << SPEED);
        panda.setArmSpeed(SPEED);

        // Init scene
        ROS_INFO("## INIT SCENE");
        panda.setScene(data_manager::get_scene(SCENE_NAME));
        ros::WallDuration(1.0).sleep();

        // Pick object
        ROS_INFO_STREAM("## PICK OBJECT to pose: " << PICK_POSE_NAME);
        panda.pick(data_manager::get_pose(PICK_POSE_NAME), 0.02);
        ros::WallDuration(1.0).sleep();

        // Place Object
        ROS_INFO("## PLACE OBJECT");
        panda.place(data_manager::get_pose(PLACE_POSE_NAME));
        ros::WallDuration(1.0).sleep();

    } catch (const my_exceptions::panda_error &e) {
        ROS_FATAL_STREAM(my_exceptions::get_err_msg(NAME, e.what()));

    } catch (const my_exceptions::data_manager_error &e) {
        ROS_FATAL_STREAM(my_exceptions::get_err_msg(NAME, e.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}