// MY LIBS
#include "arm.hpp"
#include "my_exceptions.hpp"
#include "poses_manager.hpp"

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
    ROS_INFO_STREAM(">> START: " << NAME);


    // Extract the parameters
    int SPEED;
    if (!node.getParam("speed", SPEED)) {
        ROS_FATAL_STREAM(
            ">> Can't get parameters. (Don't use rosrun. Use roslaunch)!");
        ros::shutdown();
        exit(1);
    }



    // Task
    try {
        // Create class to manage the Panda arm
        auto panda = arm::Panda("panda_arm");

        // TODO: INIT SCENE

        // Get current pose
        auto start_pose = panda.getCurrentPose();

        // Test gripper
        // panda.moveGripper();

        // Pick object
        panda.pick(poses_manager::get_pose("object"));

        // Place Object
        panda.place(poses_manager::get_pose("target"));

        // Return to start_pose
        panda.moveToPosition(start_pose, SPEED, true);

    } catch (const my_exceptions::poses_manager_error &e) {
        ROS_FATAL_STREAM(">> " << e.what());

    } catch (const my_exceptions::arm_error &e) {
        ROS_FATAL_STREAM(">> " << e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}