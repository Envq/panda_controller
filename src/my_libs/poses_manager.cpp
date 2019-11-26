#include "poses_manager.hpp"



//#############################################################################
// GLOBAL VARIABLES
const std::string PACKAGE_NAME = "panda_controller";
const std::string RELATIVE_POSES = "/db/poses.json";
std::string POSES_PATH;



//#############################################################################
// PRIVATE FUNCTIONS
std::string getPath() {
    if (POSES_PATH.empty()) {
        auto PACKAGE_PATH = ros::package::getPath(PACKAGE_NAME);

        if (PACKAGE_PATH.empty())
            throw poses_manager_error("Package path not found");

        POSES_PATH = PACKAGE_PATH + RELATIVE_POSES;
    }
    return POSES_PATH;
}



//#############################################################################
// PUBLIC FUNCTIONS IMPLEMENTATION
namespace poses_manager {

void save_pose(const std::string &NAME, const geometry_msgs::Pose &pose) {
    std::fstream file;
    Json::Value root;
    Json::StyledStreamWriter writer;

    try {
        // Init root
        file.open(getPath(), std::ios::in);
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
        file.open(getPath(), std::ios::out);
        if (!file.is_open())
            throw poses_manager_error("Can't write on path selected");

        writer.write(file, root);
        file.close();

    } catch (Json::RuntimeError &e) {
        throw poses_manager_error("Json ill-writted");
        file.close();
    }
}


geometry_msgs::Pose get_pose(const std::string &NAME) {
    std::ifstream file;
    Json::Value root;
    // Open file
    file.open(getPath(), std::ios::in);

    // Check if the file is correctly opened
    if (!file.is_open())
        throw poses_manager_error("can't open:" + getPath());


    // Get pose
    geometry_msgs::Pose pose;

    try {
        file >> root;
        pose.orientation.x = root[NAME]["orientation"]["x"].asDouble();
        pose.orientation.y = root[NAME]["orientation"]["y"].asDouble();
        pose.orientation.z = root[NAME]["orientation"]["z"].asDouble();
        pose.orientation.w = root[NAME]["orientation"]["w"].asDouble();
        pose.position.x = root[NAME]["position"]["x"].asDouble();
        pose.position.y = root[NAME]["position"]["y"].asDouble();
        pose.position.z = root[NAME]["position"]["z"].asDouble();

        file.close();

    } catch (Json::RuntimeError &e) {
        throw poses_manager_error("Json ill-writted");
        file.close();
    }
    return pose;
}


}  // namespace poses_manager