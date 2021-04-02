// ROS and Moveit
#include <ros/ros.h>

// C++
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>  //tf2::Quaternion

// BOOST
#include <boost/algorithm/string.hpp>  // split()
#include <boost/shared_ptr.hpp>

// Custom
#include "panda_controller/exceptions.hpp"
#include "panda_controller/panda_arm.hpp"
#include "panda_controller/panda_gripper.hpp"
#include "panda_controller/panda_scene.hpp"
#include "utils/colors.hpp"



// USING NAMESPACE ============================================================
using namespace panda_controller;



// CONFIGS ====================================================================
auto FG_COLOR = Colors::FG_BLUE;
auto BG_COLOR = Colors::BG_DEFAULT;
auto CMD1_COLOR = Colors::FG_GREEN;
auto CMD2_COLOR = Colors::FG_CYAN;
auto CMD3_COLOR = Colors::FG_WHITE;
auto HELP_COLOR = CMD1_COLOR;
auto ERROR_COLOR = Colors::FG_RED;
auto WARN_COLOR = Colors::FG_YELLOW;
auto CHECK_COLOR = Colors::FG_BLACK_BRIGHT;



// FUNCTIONS ==================================================================
// Check if a string is a number
bool is_number(const std::string &str) {
    // Check if is empty
    if (str.empty())
        return false;

    // Check if the first char is correct
    if (str.front() == '.' || (str.front() == '-' && str.size() == 1))
        return false;

    // Check if first and last char are correct
    if (!std::isdigit(str.back()) ||
        (str.back() != '-' && !std::isdigit(str.back())))
        return false;

    // Check the rest
    bool point_finded = false;
    for (size_t i = 1; i < str.size() - 1; i++) {
        if (!std::isdigit(str[i])) {
            // Check point symbol
            if (str[i] == '.' && !point_finded)
                point_finded = true;
            else
                return false;
        }
    }
    return true;
}


// Print commands available
void print_help() {
    std::stringstream help;
    help << Colors::BOLD << FG_COLOR << BG_COLOR;
    help << "Commands available:\n";
    help << Colors::RESET;
    help << CMD1_COLOR;
    help << "',' are automatically ignored\n\n";

    help << "[Gripper commands:]\n";
    help << "   gripper homing\n";
    help << "   gripper width\n";
    help << "   gripper 'width' 'speed'\n";
    help << "   grasp   'width' 'speed' 'force' 'epsilon_inner' "
            "'epsilon_outer'\n\n";

    help << "[Get pose of:]\n";
    help << "   get joints\n";
    help << "   get pose\n\n";

    help << "[Move absolute commands:]\n";
    help << "   move joints 'j0' 'j1' 'j2' 'j3' 'j4' 'j5' 'j6'\n";
    help << "   move pose   'px' 'py' 'pz' 'ox' 'oy' 'oz' 'ow'\n";
    help << "   move linear 'px' 'py' 'pz' 'ox' 'oy' 'oz' 'ow'\n";
    help << "   ready\n\n";

    help << "[Move relative commands:]\n";
    help << "   rel pos   'px' 'py' 'pz'\n";
    help << "   rel rpy 'roll' 'pitch' 'yaw' Note: using degree\n\n";

    help << "[convert: quaternion <-> euler in radian]\n";
    help << "   convert 'x' 'y' 'z' 'w'\n";
    help << "   convert 'roll' 'pitch' 'yaw'\n\n";

    help << "[data:]\n";
    help << "   reset scene\n";
    help << "   scene 'scene_name'\n";
    help << "   save 'pose_name'\n";
    help << "   goto 'pose_name'\n\n";

    help << "[other:]\n";
    help << "   set velocity 'vel'\n";
    help << "   quit\n";
    help << "   help\n";
    help << "-----------------------------------------------\n";
    help << Colors::RESET;

    ROS_COL_INFO(CMD3_COLOR, help.str());
}



// MAIN =======================================================================
int main(int argc, char **argv) {
    // Get the file name
    std::string file_path = argv[0];
    const std::string CURRENT_FILE_NAME =
        file_path.substr(file_path.find_last_of("/\\") + 1);


    // Setup ROS
    ros::init(argc, argv, CURRENT_FILE_NAME);
    ros::NodeHandle node("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "START NODE: ", CURRENT_FILE_NAME);


    // Extract the parameters
    bool REAL_ROBOT, ARM_ADJUST_JOINTS;
    double ARM_VELOCITY_FACTOR;
    double GRASP_SPEED, GRASP_FORCE, GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER;
    double GRIPPER_SPEED;
    double EEF_STEP, JUMP_THRESHOLD;
    if (!(node.getParam("arm_velocity_factor", ARM_VELOCITY_FACTOR) &&
          node.getParam("arm_adjust_joints", ARM_ADJUST_JOINTS) &&
          node.getParam("real_robot", REAL_ROBOT) &&
          node.getParam("gripper_speed", GRIPPER_SPEED) &&
          node.getParam("grasp_speed", GRASP_SPEED) &&
          node.getParam("grasp_force", GRASP_FORCE) &&
          node.getParam("grasp_epsilon_inner", GRASP_EPSILON_INNER) &&
          node.getParam("grasp_epsilon_outer", GRASP_EPSILON_OUTER) &&
          node.getParam("eef_step", EEF_STEP) &&
          node.getParam("jump_threshold", JUMP_THRESHOLD))) {
        ROS_FATAL_STREAM(
            get_err_msg(CURRENT_FILE_NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "PANDA CONTROLLER INITIALIZATION");
        auto arm = PandaArm();
        auto gripper = PandaGripper(REAL_ROBOT);
        auto scene = PandaScene();


        // Read command and performe task
        ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "INSERT COMMANDS: ");
        std::string command;
        while (true) {
            // get command
            std::cout << CMD1_COLOR << ">> " << CMD2_COLOR;
            std::getline(std::cin, command);
            std::cout << Colors::RESET;

            // Split of the command string
            std::vector<std::string> cmd_parts;
            boost::split(cmd_parts, command, [](char c) { return c == ' '; });

            try {
                // Check if there is a command
                if (cmd_parts.size() < 1) {
                    throw std::invalid_argument(command);
                }


                if (cmd_parts[0] == "quit") {
                    break;


                } else if (cmd_parts[0] == "help") {
                    print_help();


                } else if (cmd_parts[0] == "ready") {
                    arm.moveToReady();


                } else if (cmd_parts[0] == "set") {
                    if ((cmd_parts.size() == 3 && cmd_parts[1] == "velocity" &&
                         is_number(cmd_parts[2]))) {
                        arm.setMaxVelocityScalingFactor(
                            std::stof(cmd_parts[2]));
                    } else
                        throw std::invalid_argument(command);


                } else if (cmd_parts[0] == "reset") {
                    if ((cmd_parts.size() == 2 && cmd_parts[1] == "scene")) {
                        scene.resetScene();
                    } else
                        throw std::invalid_argument(command);


                } else if (cmd_parts[0] == "scene") {
                    if (cmd_parts.size() == 2) {
                        scene.setScene(cmd_parts[1]);
                    } else
                        throw std::invalid_argument(command);


                } else if (cmd_parts[0] == "save") {
                    if (cmd_parts.size() == 1) {
                        arm.savePose();
                    } else if (cmd_parts.size() == 2) {
                        arm.savePose(cmd_parts[1]);
                    } else
                        throw std::invalid_argument(command);


                } else if (cmd_parts[0] == "goto") {
                    if (cmd_parts.size() == 2) {
                        arm.moveToPose(cmd_parts[1]);
                    } else
                        throw std::invalid_argument(command);


                } else if (cmd_parts[0] == "get") {
                    if (cmd_parts.size() != 2) {
                        throw std::invalid_argument(command);
                    }
                    if (cmd_parts[1] == "joints") {
                        auto joints = arm.getJoints();
                        std::stringstream str;
                        str << "[";
                        for (size_t i = 0; i < joints.size() - 1; i++)
                            str << joints[i] << ", ";
                        str << joints[joints.size()] << "]";
                        ROS_COL_INFO(CMD3_COLOR, "Current joints:\n",
                                     str.str());

                    } else if (cmd_parts[1] == "pose") {
                        auto pose = arm.getPose();
                        std::stringstream str;
                        str << "[";
                        str << pose.position.x << ", ";
                        str << pose.position.y << ", ";
                        str << pose.position.z << ", ";
                        str << pose.orientation.x << ", ";
                        str << pose.orientation.y << ", ";
                        str << pose.orientation.z << ", ";
                        str << pose.orientation.w << "]\n";
                        ROS_COL_INFO(CMD3_COLOR, "Current pose:\n", str.str(),
                                     pose);
                    } else
                        throw std::invalid_argument(command);


                } else if (cmd_parts[0] == "move") {
                    if (cmd_parts.size() == 9 && is_number(cmd_parts[2]) &&
                        is_number(cmd_parts[3]) && is_number(cmd_parts[4]) &&
                        is_number(cmd_parts[5]) && is_number(cmd_parts[6]) &&
                        is_number(cmd_parts[7]) && is_number(cmd_parts[8])) {
                        if (cmd_parts[1] == "joints") {
                            std::vector<double> joints = {
                                std::stof(cmd_parts[2]),
                                std::stof(cmd_parts[3]),
                                std::stof(cmd_parts[4]),
                                std::stof(cmd_parts[5]),
                                std::stof(cmd_parts[6]),
                                std::stof(cmd_parts[7]),
                                std::stof(cmd_parts[8])};
                            arm.moveToJoints(joints, ARM_ADJUST_JOINTS);

                        } else if (cmd_parts[1] == "pose") {
                            geometry_msgs::Pose pose;
                            pose.position.x = std::stof(cmd_parts[2]);
                            pose.position.y = std::stof(cmd_parts[3]);
                            pose.position.z = std::stof(cmd_parts[4]);
                            pose.orientation.x = std::stof(cmd_parts[5]);
                            pose.orientation.y = std::stof(cmd_parts[6]);
                            pose.orientation.z = std::stof(cmd_parts[7]);
                            pose.orientation.w = std::stof(cmd_parts[8]);
                            arm.moveToPose(pose);

                        } else if (cmd_parts[1] == "linear") {
                            geometry_msgs::Pose pose;
                            pose.position.x = std::stof(cmd_parts[2]);
                            pose.position.y = std::stof(cmd_parts[3]);
                            pose.position.z = std::stof(cmd_parts[4]);
                            pose.orientation.x = std::stof(cmd_parts[5]);
                            pose.orientation.y = std::stof(cmd_parts[6]);
                            pose.orientation.z = std::stof(cmd_parts[7]);
                            pose.orientation.w = std::stof(cmd_parts[8]);
                            arm.linearMove(pose);

                        } else
                            throw std::invalid_argument(command);
                    } else
                        throw std::invalid_argument(command);


                } else if (cmd_parts[0] == "rel") {
                    if (cmd_parts.size() == 5 && is_number(cmd_parts[2]) &&
                        is_number(cmd_parts[3]) && is_number(cmd_parts[4])) {
                        if (cmd_parts[1] == "pos") {
                            arm.relativeMovePos(std::stof(cmd_parts[2]),
                                                std::stof(cmd_parts[3]),
                                                std::stof(cmd_parts[4]));

                        } else if (cmd_parts[1] == "rpy") {
                            arm.relativeMoveRPY(std::stof(cmd_parts[2]),
                                                std::stof(cmd_parts[3]),
                                                std::stof(cmd_parts[4]));

                        } else
                            throw std::invalid_argument(command);
                    } else
                        throw std::invalid_argument(command);


                } else if (cmd_parts[0] == "convert") {
                    if (cmd_parts.size() == 5 && is_number(cmd_parts[1]) &&
                        is_number(cmd_parts[2]) && is_number(cmd_parts[3]) &&
                        is_number(cmd_parts[4])) {
                        // Get quaternion
                        tf2::Quaternion quat = {
                            std::stof(cmd_parts[1]), std::stof(cmd_parts[2]),
                            std::stof(cmd_parts[3]), std::stof(cmd_parts[4])};
                        // Convert into RPY
                        double roll, pitch, yaw;
                        tf2::Matrix3x3 matrix(quat);
                        matrix.getRPY(roll, pitch, yaw);
                        // Create string
                        std::stringstream str;
                        str << "[";
                        str << roll << ", ";
                        str << pitch << ", ";
                        str << yaw << "]";
                        ROS_COL_INFO(CMD3_COLOR,
                                     "Convert into Eular RPY (Radian):\n",
                                     str.str());

                    } else if (cmd_parts.size() == 4 &&
                               is_number(cmd_parts[1]) &&
                               is_number(cmd_parts[2]) &&
                               is_number(cmd_parts[3])) {
                        // Get RPY
                        tf2::Quaternion quat;
                        quat.setRPY(std::stof(cmd_parts[1]),
                                    std::stof(cmd_parts[2]),
                                    std::stof(cmd_parts[3]));
                        // Create string
                        std::stringstream str;
                        str << "[";
                        str << quat.getX() << ", ";
                        str << quat.getY() << ", ";
                        str << quat.getZ() << ", ";
                        str << quat.getW() << "]";
                        ROS_COL_INFO(CMD3_COLOR,
                                     "Convert into Quaternion (Radian):\n",
                                     str.str());
                    } else
                        throw std::invalid_argument(command);


                } else if (cmd_parts[0] == "gripper") {
                    if (cmd_parts.size() == 2) {
                        if (cmd_parts[1] == "homing") {
                            gripper.homing();
                        } else if (cmd_parts[1] == "width") {
                            ROS_COL_INFO(CMD3_COLOR, "gripper width:\n",
                                         gripper.getWidth());
                        } else if (is_number(cmd_parts[1])) {
                            gripper.move(std::stod(cmd_parts[1]),
                                         GRIPPER_SPEED);
                        } else
                            throw std::invalid_argument(command);

                    } else if (cmd_parts.size() == 3 &&
                               is_number(cmd_parts[1]) &&
                               is_number(cmd_parts[2])) {
                        gripper.move(std::stod(cmd_parts[1]),
                                     std::stod(cmd_parts[2]));
                    } else
                        throw std::invalid_argument(command);
                } else if (cmd_parts[0] == "grasp") {
                    if (cmd_parts.size() == 2 && is_number(cmd_parts[1])) {
                        gripper.grasp(std::stod(cmd_parts[1]), GRASP_SPEED,
                                      GRASP_FORCE, GRASP_EPSILON_INNER,
                                      GRASP_EPSILON_OUTER);

                    } else if (cmd_parts.size() == 3 &&
                               is_number(cmd_parts[1]) &&
                               is_number(cmd_parts[2])) {
                        gripper.grasp(std::stod(cmd_parts[1]),
                                      std::stod(cmd_parts[2]), GRASP_FORCE,
                                      GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER);

                    } else if (cmd_parts.size() == 4 &&
                               is_number(cmd_parts[1]) &&
                               is_number(cmd_parts[2]) &&
                               is_number(cmd_parts[3])) {
                        gripper.grasp(std::stod(cmd_parts[1]),
                                      std::stod(cmd_parts[2]),
                                      std::stod(cmd_parts[3]),
                                      GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER);

                    } else if (cmd_parts.size() == 5 &&
                               is_number(cmd_parts[1]) &&
                               is_number(cmd_parts[2]) &&
                               is_number(cmd_parts[3]) &&
                               is_number(cmd_parts[4])) {
                        gripper.grasp(
                            std::stod(cmd_parts[1]), std::stod(cmd_parts[2]),
                            std::stod(cmd_parts[3]), std::stod(cmd_parts[4]),
                            GRASP_EPSILON_OUTER);

                    } else if (cmd_parts.size() == 6 &&
                               is_number(cmd_parts[1]) &&
                               is_number(cmd_parts[2]) &&
                               is_number(cmd_parts[3]) &&
                               is_number(cmd_parts[4]) &&
                               is_number(cmd_parts[5])) {
                        gripper.grasp(
                            std::stod(cmd_parts[1]), std::stod(cmd_parts[2]),
                            std::stod(cmd_parts[3]), std::stod(cmd_parts[4]),
                            std::stod(cmd_parts[5]));
                    } else
                        throw std::invalid_argument(command);
                } else
                    throw std::invalid_argument(command);
            } catch (const std::invalid_argument &e) {
                ROS_WARN_STREAM("invalid command: " << e.what());
            } catch (const DataManagerErr &e) {
                ROS_WARN_STREAM("invalid command: " << e.what());
            }
        }
    } catch (const PandaControllerErr &err) {
        ROS_FATAL_STREAM(get_err_msg(CURRENT_FILE_NAME, err.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}
