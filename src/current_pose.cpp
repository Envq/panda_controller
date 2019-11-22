// ROS
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>



//#############################################################################
int main(int argc, char **argv) {
    // Get the file name
    std::string file_path = argv[0];
    const std::string NAME =
        file_path.substr(file_path.find_last_of("/\\") + 1);


    // Setup ROS
    ros::init(argc, argv, NAME);
    ros::NodeHandle node("/panda_ws");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ROS_INFO_STREAM("START: " << NAME);


    // Create the current-pose publisher
    ros::Publisher pub = node.advertise<geometry_msgs::Pose>("current_pose", 1000);


    // Task
    try {
        static const std::string PLANNING_GROUP = "panda_arm";
        moveit::planning_interface::MoveGroupInterface move_group(
            PLANNING_GROUP);

        // get current pose
        geometry_msgs::Pose pose = move_group.getCurrentPose().pose;

        // Publish pose
        pub.publish(pose);
        ROS_INFO("Object Pose published");
        ROS_INFO_STREAM(pose);

    } catch (std::runtime_error &e) {
        ROS_ERROR("Object Pose can't be published");
    }


    // Finish
    ros::shutdown();
    return 0;
}
