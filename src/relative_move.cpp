// MY LIBS
#include "arm.hpp"
#include "my_exceptions.hpp"

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
    float X, Y, Z, SPEED;
    bool PLAN_ONLY;
    if (!node.getParam("x", X) || !node.getParam("y", Y) ||
        !node.getParam("z", Z) || !node.getParam("speed", SPEED) ||
        !node.getParam("plan_only", PLAN_ONLY)) {
        ROS_FATAL_STREAM(
            ">> Can't get parameters. (Don't use rosrun. Use roslaunch)!");
        ros::shutdown();
        return 0;
    }

    // Task
    try {
        // Create class to manage the Panda arm
        auto panda = arm::Panda();

        // Set speed
        panda.setSpeed(SPEED);

        // Create new pose
        auto pose = panda.getCurrentPose();
        pose.position.x += X;
        pose.position.y += Y;
        pose.position.z += Z;

        // Move arm
        panda.moveToPosition(pose, PLAN_ONLY);

    } catch (const my_exceptions::arm_error &e) {
        ROS_FATAL_STREAM(">> " << e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}