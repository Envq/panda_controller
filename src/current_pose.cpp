// MY LIBS
#include "data_manager.hpp"
#include "my_exceptions.hpp"

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
    ROS_INFO_STREAM(">> START: " << NAME);


    // Extract the parameters
    std::string POSE_NAME;
    if (!node.getParam("name", POSE_NAME)) {
        ROS_FATAL_STREAM(
            ">> Can't get parameters. (Don't use rosrun. Use roslaunch)!");
        ros::shutdown();
        return 0;
    }

    // Task
    try {
        // Create panda interface
        moveit::planning_interface::MoveGroupInterface move_group(PANDA_GROUP);

        // Get current pose
        geometry_msgs::Pose pose;
        pose = move_group.getCurrentPose().pose;

        // Print current pose
        ROS_INFO_STREAM("CURRENT POSE:\n" << pose);

        // Save pose in json file
        data_manager::save_pose(POSE_NAME, pose);

    } catch (const my_exceptions::data_manager_error &e) {
        ROS_FATAL_STREAM(">> " << e.what());

    } catch (const std::runtime_error &e) {
        ROS_FATAL(">> Impossible initialize moveGroupInterface");
    }


    // Finish
    ros::shutdown();
    return 0;
}