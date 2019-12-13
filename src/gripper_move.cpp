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
    bool HOMING, MOVE, GRASP;
    float WIDTH, SPEED, FORCE, EPSILON_INNER, EPSILON_OUTER;
    if (!(node.getParam("homing", HOMING) && node.getParam("move", MOVE) &&
          node.getParam("grasp", GRASP) && node.getParam("width", WIDTH) &&
          node.getParam("speed", SPEED) && node.getParam("force", FORCE) &&
          node.getParam("epsilon_inner", EPSILON_INNER) &&
          node.getParam("epsilon_outer", EPSILON_OUTER))) {
        ROS_FATAL_STREAM(">> [" << NAME << "] Can't get parameters");
        ros::shutdown();
        return 0;
    }

    // Task
    try {
        // Create class to manage the Panda arm
        ROS_INFO("## INIT PANDA CONTROLLER");
        auto panda = robot::Panda();

        // Execute gripper homing
        if (HOMING) {
            ROS_INFO_STREAM("## GRIPPER HOMING");
            panda.gripperHoming();
        }

        // Execute gripper move
        if (MOVE) {
            ROS_INFO_STREAM("## GRIPPER MOVE:\n"
                            << "Width: " << WIDTH << "\nSpeed: " << SPEED);
            panda.gripperMove(WIDTH, SPEED);
        }

        // Execute gripper grasp
        if (GRASP) {
            ROS_INFO_STREAM("## GRIPPER MOVE:\n"
                            << "Width: " << WIDTH << "\nSpeed: " << SPEED);
            panda.gripperMove(WIDTH, SPEED);
        }

    } catch (const my_exceptions::panda_error &e) {
        ROS_FATAL_STREAM(">> [" << NAME << "] >> panda_error >> " << e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}