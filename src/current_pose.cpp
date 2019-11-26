// ROS
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>

// C++
#include <fstream>
#include <jsoncpp/json/json.h>



//#############################################################################
// GLOBAL VARIABLE
static const std::string PANDA_GROUP = "panda_arm";
const std::string PATH_POSES =
    "/home/envq/panda_ws/src/panda_controller/db/poses.json";



//#############################################################################
// PRIVATE FUNCTIONS
void save_pose(const geometry_msgs::Pose pose, const std::string name);



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


    // Extract the parameter
    std::string pose_name;
    if (!node.getParam("name", pose_name)) {
        ROS_FATAL_STREAM(">> Use launch file!");
        ros::shutdown();
        exit(1);
    }


    // Task
    try {
        // Create panda interface
        moveit::planning_interface::MoveGroupInterface move_group(PANDA_GROUP);

        // Get current pose
        geometry_msgs::Pose pose;
        pose = move_group.getCurrentPose().pose;
        // DEBUG
        // pose.orientation.x = 0.0;
        // pose.orientation.y = 0.0;
        // pose.orientation.z = 0.0;
        // pose.orientation.w = 0.0;
        // pose.position.x = 0.0;
        // pose.position.y = 0.5;
        // pose.position.z = 0.0;

        // Print current pose
        ROS_INFO_STREAM(pose);

        // Save pose in json file
        save_pose(pose, pose_name);

    } catch (const std::runtime_error &e) {
        ROS_FATAL(">> Impossible get current position");

    } catch (Json::RuntimeError &e) {
        ROS_FATAL(">> Json ill-writted");
    }


    // Finish
    ros::shutdown();
    return 0;
}



//#############################################################################
// IMPLEMENTATIONS
void save_pose(const geometry_msgs::Pose pose, const std::string name) {
    std::fstream file;
    Json::Value root;
    Json::StyledStreamWriter writer;

    // Init root
    file.open(PATH_POSES, std::ios::in);
    if (file.is_open()) {
        file >> root;
        file.close();
    }

    // Update root
    root[name]["orientation"]["x"] = pose.orientation.x;
    root[name]["orientation"]["y"] = pose.orientation.y;
    root[name]["orientation"]["z"] = pose.orientation.z;
    root[name]["orientation"]["w"] = pose.orientation.w;
    root[name]["position"]["x"] = pose.position.x;
    root[name]["position"]["y"] = pose.position.y;
    root[name]["position"]["z"] = pose.position.z;

    // Save the pose
    file.open(PATH_POSES, std::ios::out);
    writer.write(file, root);
    file.close();
}