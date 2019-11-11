#include <ros/ros.h>


int main(int argc, char **argv) {
    // Get the file name
    std::string file_path = argv[0];
    const std::string NAME =
        file_path.substr(file_path.find_last_of("/\\") + 1);


    // Setup ROS
    ROS_INFO_STREAM("START: " << NAME);
    ros::init(argc, argv, NAME);
    ros::NodeHandle node_handle("~");  // private namespace
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // Finish
    ros::shutdown();
    return 0;
}
