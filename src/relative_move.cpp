// MY LIBS
#include "arm.hpp"
#include "my_exceptions.hpp"

// ROS
#include <ros/ros.h>



//#############################################################################
// GLOBAL VARIABLE
static const std::string PANDA_GROUP = "panda_arm";


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
    float X, Y, Z, SPEED;
    if (!node.getParam("x", X) || !node.getParam("y", Y) ||
        !node.getParam("z", Z) || !node.getParam("speed", SPEED)) {
        ROS_FATAL_STREAM(
            ">> Can't get parameters. (Don't use rosrun. Use roslaunch)!");
        ros::shutdown();
        return 0;
    }

    // Task
    try {
        // Create class to manage the Panda arm
        auto panda = arm::Panda("panda_arm");

        // Create new pose
        auto pose = panda.getCurrentPose();
        pose.position.x += X;
        pose.position.y += Y;
        pose.position.z += Z;

        // Move arm
        panda.moveToPosition(pose, SPEED, true);

    } catch (const my_exceptions::arm_error &e) {
        ROS_FATAL_STREAM(">> " << e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}