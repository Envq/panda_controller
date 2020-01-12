// PANDA CONTROLLER
#include "data_manager.hpp"
#include "my_exceptions.hpp"
#include "panda.hpp"

// ROS
#include <ros/ros.h>

// BOOST
#include <boost/algorithm/string.hpp>

// C++
#include <iostream>
#include <vector>



//#############################################################################
// DEFAULT VALUES #############################################################
std::string POSE_NAME_DFLT;
float GRIPPER_SPEED_DFLT, GRIPPER_FORCE_DFLT, GRIPPER_EPSILON_INNER_DFLT,
    GRIPPER_EPSILON_OUTER_DFLT;



//#############################################################################
// PRIVATE FUNCTIONS ##########################################################
// Check if a string is a number
bool is_number(const std::string &str);

// Perform parsing of readed command and run it
void run_command(robot::Panda &panda, const std::string &command);



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
    ROS_INFO_STREAM("## START: " << NAME);


    // Extract the parameters
    if (!(node.getParam("pose_name", POSE_NAME_DFLT) &&
          node.getParam("gripper_speed", GRIPPER_SPEED_DFLT) &&
          node.getParam("gripper_force", GRIPPER_FORCE_DFLT) &&
          node.getParam("gripper_epsilon_inner", GRIPPER_EPSILON_INNER_DFLT) &&
          node.getParam("gripper_epsilon_outer", GRIPPER_EPSILON_OUTER_DFLT))) {
        ROS_FATAL_STREAM(
            my_exceptions::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_INFO("## INIT PANDA CONTROLLER");
        auto panda = robot::Panda();

        // Read command and performe task
        std::string command;
        ROS_INFO("Insert commands:");
        while (command != "quit") {
            std::cout << "\033[1;32m"
                      << ">> "
                      << "\033[0m";

            try {
                std::getline(std::cin, command);  // Read stdin
                run_command(panda, command);      // Perform task

            } catch (const my_exceptions::panda_error &e) {
                ROS_ERROR_STREAM(my_exceptions::get_err_msg(NAME, e.what()));

            } catch (const my_exceptions::data_manager_error &e) {
                ROS_ERROR_STREAM(my_exceptions::get_err_msg(NAME, e.what()));
            }

            std::cout << "\033[1;32m"
                      << "-------------------------------------"
                      << "\033[0m" << std::endl
                      << std::endl;
        }
    } catch (const my_exceptions::panda_error &e) {
        ROS_FATAL_STREAM(my_exceptions::get_err_msg(NAME, e.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}



//#############################################################################
// PRIVATE FUNCTIONS IMPLEMENTATIONS
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
    for (int i = 1; i < str.size() - 1; i++) {
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


void run_command(robot::Panda &panda, const std::string &command) {
    // Parse of the command string
    std::vector<std::string> cmd;
    boost::split(cmd, command, [](char c) { return c == ' '; });

    try {
        // Check if there is a command
        if (cmd.size() < 1)
            throw std::invalid_argument(command);

        // CASE QUIT
        if (cmd[0] == "quit") {
            if ((cmd.size() != 1))
                throw std::invalid_argument(command);

            ROS_INFO("## SELECTED QUIT");

            // CASE HELP
        } else if (cmd[0] == "help") {
            if ((cmd.size() != 1))
                throw std::invalid_argument(command);

            ROS_INFO_STREAM(
                "## SELECTED HELP\n"
                << "Leggends:\n"
                << " - [] indicates a parameter\n"
                << " - () indicates a optional parameter with default "
                   "value in "
                   "launch file\n"
                << "Commands available:\n"
                << " - help: to see the list of commands\n"
                << " - quit: to close the node\n"
                << " - speed [value]: to set the arm speed value\n"
                << " - save [(name)]: to save the current pose with the "
                   "specified name\n"
                << " - move offset [x y z]: to move the arm along the "
                   "x,y,z "
                   "specified directions in meters\n"
                << " - move pose [name]: to move the arm on the specified "
                   "pose "
                   "saved in database\n"
                << " - move gripper [width (speed)]: to move the gripper "
                   "fingers "
                   "with the specified speed on the specified width from "
                   "center\n"
                << " - homing arm: to do the homing of the arm\n"
                << " - homing gripper: to do the homing of the gripper\n"
                << " - grasp [width (speed force epsilon_inner "
                   "epsilon_outer)]: to perform the grasp of gripper");

            // CASE SPEED
        } else if (cmd[0] == "speed") {
            if ((cmd.size() != 2) || !is_number(cmd[1]))
                throw std::invalid_argument(command);

            float speed = std::stof(cmd[1]);
            ROS_INFO_STREAM("## SET ARM SPEED TO: " << speed);
            panda.setArmSpeed(speed);

            // CASE SAVE
        } else if (cmd[0] == "save") {
            if (cmd.size() > 2)
                throw std::invalid_argument(command);

            std::string pose_name = (cmd.size() == 1) ? POSE_NAME_DFLT : cmd[1];
            auto pose = panda.getCurrentPose();
            ROS_INFO_STREAM("## SAVE POSE: " << pose_name << std::endl << pose);
            data_manager::save_pose(pose_name, pose);

            // CASE MOVE
        } else if (cmd[0] == "move") {
            if (cmd.size() < 2)
                throw std::invalid_argument(command);

            if (cmd[1] == "offset") {
                if ((cmd.size() != 5) || !is_number(cmd[2]) ||
                    !is_number(cmd[3]) || !is_number(cmd[4]))
                    throw std::invalid_argument(command);

                float x = std::stof(cmd[2]);
                float y = std::stof(cmd[3]);
                float z = std::stof(cmd[4]);
                auto target_pose = panda.getCurrentPose();
                target_pose.position.x += x;
                target_pose.position.y += y;
                target_pose.position.z += z;
                ROS_INFO("## SELECTED OFFSET MODE");
                ROS_INFO_STREAM("## MOVE TO POSE:\n" << target_pose);
                panda.moveToPosition(target_pose);

            } else if (cmd[1] == "pose") {
                if (cmd.size() != 3)
                    throw std::invalid_argument(command);

                ROS_INFO("## SELECTED POSE MODE");
                ROS_INFO_STREAM("## GET POSE: " << cmd[2]);
                auto target_pose = data_manager::get_pose(cmd[2]);
                ROS_INFO_STREAM("## MOVE TO POSE:\n" << target_pose);
                panda.moveToPosition(target_pose);

            } else if (cmd[1] == "gripper") {
                if ((cmd.size() != 3 && cmd.size() != 4) ||
                    !is_number(cmd[2]) ||
                    (cmd.size() == 4 && !is_number(cmd[3])))
                    throw std::invalid_argument(command);

                ROS_INFO("## SELECTED GRIPPER MODE");
                float width = std::stof(cmd[2]);
                float speed =
                    (cmd.size() == 3) ? GRIPPER_SPEED_DFLT : std::stof(cmd[3]);
                ROS_INFO_STREAM("## MOVE GRIPPER TO: " << width);
                panda.gripperMove(width, speed);

            } else {
                throw std::invalid_argument(command);
            }

            // CASE HOMING
        } else if (cmd[0] == "homing") {
            if (cmd.size() != 2)
                throw std::invalid_argument(command);

            if (cmd[1] == "arm") {
                ROS_INFO("## SELECTED ARM MODE");
                try {
                    auto target_pose = data_manager::get_pose("ready");
                    ROS_INFO_STREAM("## MOVE TO POSE:\n" << target_pose);
                    panda.moveToPosition(target_pose);
                } catch (const my_exceptions::data_manager_error &e) {
                    ROS_FATAL_STREAM("Invalid pose name: " << cmd[2]);
                }

            } else if (cmd[1] == "gripper") {
                ROS_INFO("## SELECTED GRIPPER MODE");
                panda.gripperHoming();

            } else {
                throw std::invalid_argument(command);
            }

            // CASE GRASP
        } else if (cmd[0] == "grasp") {
            if ((cmd.size() != 2 && cmd.size() != 6) || !is_number(cmd[1]) ||
                (cmd.size() == 6 && !(is_number(cmd[2]) && is_number(cmd[3]) &&
                                      is_number(cmd[4]) && is_number(cmd[5]))))
                throw std::invalid_argument(command);

            float width = std::stof(cmd[1]);
            float speed, force, epsilon_inner, epsilon_outer;
            if (cmd.size() == 6) {
                speed = std::stof(cmd[2]);
                force = std::stof(cmd[3]);
                epsilon_inner = std::stof(cmd[4]);
                epsilon_outer = std::stof(cmd[5]);
            } else {
                speed = GRIPPER_SPEED_DFLT;
                force = GRIPPER_FORCE_DFLT;
                epsilon_inner = GRIPPER_EPSILON_INNER_DFLT;
                epsilon_outer = GRIPPER_EPSILON_OUTER_DFLT;
            }
            ROS_INFO_STREAM("## GRASP WITH:"
                            << "\n - width: " << width << "\n - speed: "
                            << speed << "\n - force: " << force
                            << "\n - epsilon_inner: " << epsilon_inner
                            << "\n - epsilon_outer: " << epsilon_outer);
            panda.gripperGrasp(width, speed, force, epsilon_inner,
                               epsilon_outer);

        } else {
            throw std::invalid_argument(command);
        }

    } catch (const std::invalid_argument &e) {
        ROS_WARN_STREAM("invalid command: " << e.what());
    }
}