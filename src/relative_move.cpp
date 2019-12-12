// MY LIBS
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
    float X, Y, Z, GRIPPER, SPEED;
    bool PLAN_ONLY;
    if (!(node.getParam("x", X) && node.getParam("y", Y) &&
          node.getParam("z", Z) && node.getParam("gripper", GRIPPER) &&
          node.getParam("speed", SPEED) &&
          node.getParam("plan_only", PLAN_ONLY))) {
        ROS_FATAL_STREAM(">> [" << NAME << "] Can't get parameters");
        ros::shutdown();
        return 0;
    }

    // Task
    try {
        // Create class to manage the Panda arm
        ROS_INFO("## INIT PANDA CONTROLLER");
        auto panda = robot::Panda();

        // Set robot speed
        ROS_INFO_STREAM("## SET SPEED: " << SPEED);
        panda.setSpeed(SPEED);

        // Move gripper
        panda.moveGripper(GRIPPER);

        // Create new pose
        ROS_INFO_STREAM("## NEW POSE: ");
        auto target_pose = panda.getCurrentPose();
        target_pose.position.x += X;
        target_pose.position.y += Y;
        target_pose.position.z += Z;
        ROS_INFO_STREAM(target_pose);

        // Move arm
        ROS_INFO_STREAM("## MOVE TO POSE");
        panda.moveToPosition(target_pose, PLAN_ONLY);


    } catch (const my_exceptions::panda_error &e) {
        ROS_FATAL_STREAM(">> [" << NAME << "] >> panda_error >> " << e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}