// PANDA CONTROLLER
#include "colors.hpp"  //ROS_STRONG_INFO
#include "data_manager.hpp"
#include "exceptions.hpp"  //PCEXC
#include "panda.hpp"

// ROS
#include <ros/ros.h>



//##############################################################################
// DEFAULT VALUES ##############################################################
const auto FG_COLOR = Colors::FG_BLUE;
const auto BG_COLOR = Colors::BG_BLACK;



//#############################################################################
// MAIN #######################################################################
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
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "START NODE: ", NAME);


    try {
        // moveit::planning_interface::MoveGroupInterface hand("hand");
        // std::vector<double> joints;
        // joints.resize(2);
        // // hand.setNamedTarget("close");
        // joints[0] = 0.0;
        // joints[1] = 0.0;
        // hand.setJointValueTarget(joints);
        // hand.move();
        // ros::WallDuration(1.0).sleep();
        // // hand.setNamedTarget("open");
        // joints[0] = 0.035;
        // joints[1] = 0.035;
        // hand.setJointValueTarget(joints);
        // hand.move();
        // ros::WallDuration(1.0).sleep();


        // std::string LINK;
        // node.getParam("link", LINK);
        // auto panda = robot::Panda(false);
        // ROS_INFO_STREAM(panda.getLinkNames());
        // ROS_WARN_STREAM(LINK);
        // panda.setScene(data_manager::get_scene("test"));
        // panda.moveToPose(data_manager::get_pose("test"));


        // std::string LINK1, LINK2;
        // node.getParam("link1", LINK1);
        // node.getParam("link2", LINK2);
        // auto panda = robot::Panda(false);
        // panda.setEndEffectorLink(LINK1);
        // ROS_WARN_STREAM(LINK1);
        // panda.moveToPose(data_manager::get_pose("test"));
        // panda.setEndEffectorLink(LINK2);
        // ROS_WARN_STREAM(LINK2);
        // panda.moveToPose(data_manager::get_pose("test"));


        auto panda = robot::Panda(false);
        ROS_INFO_STREAM(panda.getJointNames());
        ROS_INFO_STREAM(panda.getLinkNames());

        // std::string LINK1, LINK2;
        // node.getParam("link1", LINK1);
        // node.getParam("link2", LINK2);
        // moveit::planning_interface::MoveGroupInterface panda("panda_arm");
        // ROS_WARN_STREAM(panda.setEndEffectorLink(LINK1));
        // ROS_WARN_STREAM(LINK1);
        // panda.setPoseTarget(data_manager::get_pose("test"));
        // panda.move();
        // panda.clearPoseTargets();
        // ROS_WARN_STREAM(panda.setEndEffectorLink(LINK2));
        // ROS_WARN_STREAM(LINK2);
        // panda.setPoseTarget(data_manager::get_pose("test"));
        // panda.move();
        // panda.clearPoseTargets();


        // moveit::planning_interface::MoveGroupInterface panda("panda_arm");
        // for (auto link : panda.getLinkNames()) {
        //     ROS_INFO_STREAM(link);
        // }
        // for (auto joint : panda.getJointNames()) {
        //     ROS_INFO_STREAM(joint);
        // }


        // std::string LINK;
        // double R, P, Y;
        // node.getParam("link", LINK);
        // node.getParam("r", R);
        // node.getParam("p", P);
        // node.getParam("y", Y);
        // auto panda = robot::Panda(false);
        // ROS_INFO_STREAM(panda.getLinkNames());
        // panda.setScene(data_manager::get_scene("test"));

        // auto target_pose = panda.getCurrentPose(LINK);
        // tf2::Quaternion original, orientation, rotation;
        // target_pose.position.z = 0.3;
        // tf2::convert(target_pose.orientation,
        //              original);  // get original orientation
        // rotation.setRPY(R * M_PI / 180.0, P * M_PI / 180.0,
        //                 Y * M_PI / 180.0);  // get rotation orientation
        // orientation = rotation * original;  // get new orientation
        // orientation.normalize();            // normalize new orientation
        // target_pose.orientation = tf2::toMsg(orientation);  // Update

        // ros::WallDuration(2.0).sleep();
        // panda.moveToPose(target_pose, LINK);

    } catch (const PCEXC::PandaControllerException &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }



    // Finish
    ros::shutdown();
    return 0;
}