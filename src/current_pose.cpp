// MY LIBS
#include "data_manager.hpp"
#include "my_exceptions.hpp"
#include "panda.hpp"

// ROS
#include <ros/ros.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>



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
    ROS_INFO_STREAM("## START: " << NAME);


    // Extract the parameters
    std::string POSE_NAME;
    if (!node.getParam("name", POSE_NAME)) {
        ROS_FATAL_STREAM(">> [" << NAME << "] Can't get parameters");
        ros::shutdown();
        return 0;
    }

    // Task
    try {
        // Create class to manage the Panda arm
        ROS_INFO("## INIT PANDA CONTROLLER");
        auto panda = robot::Panda();

        // Get current pose
        ROS_INFO("## GET CURRENT POSE");
        auto pose = panda.getCurrentPose();
        ROS_INFO_STREAM("## POSE: \n" << pose);

        // Save pose in json file
        ROS_INFO("## SAVE POSE");
        data_manager::save_pose(POSE_NAME, pose);


    } catch (const my_exceptions::panda_error &e) {
        ROS_FATAL_STREAM(">> [" << NAME << "] >> panda_error >> " << e.what());

    } catch (const my_exceptions::data_manager_error &e) {
        ROS_FATAL_STREAM(">> [" << NAME << "] >> data_manager_error >> "
                                << e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}