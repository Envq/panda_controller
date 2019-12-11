// MY LIBS
#include "data_manager.hpp"
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
    std::string POSE;
    float SPEED;
    bool PLAN_ONLY;
    if (!(node.getParam("pose", POSE) && node.getParam("speed", SPEED) &&
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

        // Extract pose
        ROS_INFO_STREAM("## GET POSE");
        auto target_pose = data_manager::get_pose(POSE);

        // Move arm
        ROS_INFO_STREAM("## MOVE TO POSE");
        panda.moveToPosition(target_pose, PLAN_ONLY);


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