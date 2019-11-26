// C++
#include <fstream>
#include <jsoncpp/json/json.h>

// ROS
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>



//#############################################################################
// GLOBAL VARIABLE
static const std::string PANDA_GROUP = "panda_arm";



//#############################################################################
// PRIVATE FUNCTIONS
void save_pose(const std::string PATH, const std::string NAME,
               const geometry_msgs::Pose pose);



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
    std::string POSES_PATH;
    if (!node.getParam("name", POSE_NAME) ||
        !node.getParam("poses_path", POSES_PATH)) {
        ROS_FATAL_STREAM(
            ">> Can't get parameters. (Don't use rosrun. Use roslaunch)!");
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

        // Print current pose
        ROS_INFO_STREAM("CURRENT POSE:\n" << pose);

        // Save pose in json file
        save_pose(POSES_PATH, POSE_NAME, pose);

    } catch (const std::runtime_error &e) {
        ROS_FATAL(">> Impossible get current position");

    } catch (Json::RuntimeError &e) {
        ROS_FATAL(">> Json ill-writted");

    } catch (std::invalid_argument &e) {
        ROS_FATAL_STREAM(">> " << e.what());
    }


    // Finish
    ros::shutdown();
    return 0;
}



//#############################################################################
// IMPLEMENTATIONS
void save_pose(const std::string PATH, const std::string NAME,
               const geometry_msgs::Pose pose) {
    std::fstream file;
    Json::Value root;
    Json::StyledStreamWriter writer;

    // Init root
    file.open(PATH, std::ios::in);
    if (file.is_open()) {
        file >> root;
        file.close();
    }

    // Update root
    root[NAME]["orientation"]["x"] = pose.orientation.x;
    root[NAME]["orientation"]["y"] = pose.orientation.y;
    root[NAME]["orientation"]["z"] = pose.orientation.z;
    root[NAME]["orientation"]["w"] = pose.orientation.w;
    root[NAME]["position"]["x"] = pose.position.x;
    root[NAME]["position"]["y"] = pose.position.y;
    root[NAME]["position"]["z"] = pose.position.z;

    // Save the pose
    file.open(PATH, std::ios::out);
    if (!file.is_open())
        throw std::invalid_argument("Can't write on path selected");

    writer.write(file, root);
    file.close();
}