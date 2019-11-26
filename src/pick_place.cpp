// ROS
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>



//#############################################################################
// CALLBACK
void obj_pose_callback(const geometry_msgs::Pose &pose) {
    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Print Pose readed
    ROS_INFO_STREAM(pose);

    // Set the target Pose
    move_group.setPoseTarget(pose);

    // Perform the planning and then make the move if successful
    bool success = (move_group.plan(my_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        move_group.move();
        ROS_INFO("Panda pose goal: SUCCESS");
    } else {
        ROS_ERROR("Panda pose goal: FAILURE");
    }

    ROS_INFO("FINISHED TASK");
}



//#############################################################################
int main(int argc, char **argv) {
    // Get the file name
    std::string file_path = argv[0];
    const std::string NAME =
        file_path.substr(file_path.find_last_of("/\\") + 1);


    // Setup ROS
    ros::init(argc, argv, NAME);
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ROS_INFO_STREAM("START: " << NAME);


    // Subscriber to get the object position
    ros::Subscriber sub = node.subscribe("obj_pose", 1000, obj_pose_callback);


    // Finish
    ros::waitForShutdown();
    return 0;
}