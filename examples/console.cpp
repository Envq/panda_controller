/**
 * @file console.cpp
 * @author Enrico Sgarbanti
 * @brief Easy console to manage panda robot
 * @version 0.1
 * @date 20-02-2020
 *
 * @copyright Copyright (c) 2020 by Enrico Sgarbanti. License GPLv3.
 *
 */
// PANDA CONTROLLER
#include "colors.hpp"  //ROS_STRONG_INFO
#include "data_manager.hpp"
#include "exceptions.hpp"  //PCEXC
#include "panda.hpp"

// ROS
#include <ros/ros.h>

// BOOST
#include <boost/algorithm/string.hpp>  // split()
#include <boost/shared_ptr.hpp>

// C++
#include <iostream>
#include <vector>



//#############################################################################
// CONFIGS ####################################################################
namespace config {
const int HISTORY_MAX_SIZE = 20;
const auto FG = Colors::FG_BLUE;
const auto BG = Colors::BG_BLACK;
float GRASP_FORCE, GRASP_EPSILON_INNER, GRASP_EPSILON_OUTER, CARTESIAN_STEP,
    CARTESIAN_JUMP;
bool GRIPPER_IS_ACTIVE, JOINT_ADJUST_IN_BOUNDS;
}  // namespace config



//#############################################################################
// PRIVATE FUNCTIONS AND CLASSES ##############################################
// Perform parsing of readed command and run it
void run_command(robot::Panda &panda, const std::string &command);

// Check if a string is a number
bool is_number(const std::string &str);

// Manage history of commands
class History {
  private:
    size_t max_size_;
    std::vector<std::string> history_;
    size_t cursor_;

  public:
    explicit History(const int &SIZE);
    void setSize(const int &SIZE);
    void add(const std::string &COMMAND);
    std::string prev();
    std::string next();
};



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
    ROS_STRONG_INFO(config::FG, config::BG, "START NODE: ", NAME);


    // Extract the parameters
    if (!(node.getParam("gripper_is_active", config::GRIPPER_IS_ACTIVE) &&
          node.getParam("grasp_force", config::GRASP_FORCE) &&
          node.getParam("grasp_epsilon_inner", config::GRASP_EPSILON_INNER) &&
          node.getParam("grasp_epsilon_outer", config::GRASP_EPSILON_OUTER) &&
          node.getParam("cartesian_step", config::CARTESIAN_STEP) &&
          node.getParam("cartesian_jump", config::CARTESIAN_JUMP) &&
          node.getParam("joint_adjust_in_bounds",
                        config::JOINT_ADJUST_IN_BOUNDS))) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_STRONG_INFO(config::FG, config::BG,
                        "PANDA CONTROLLER INITIALIZATION");
        auto panda = robot::Panda(config::GRIPPER_IS_ACTIVE);

        // Read command and performe task
        std::string command;
        boost::shared_ptr<History> cmd_history(
            new History(config::HISTORY_MAX_SIZE));  // Create history
        ROS_STRONG_INFO(config::FG, config::BG, "INSERT COMMANDS: ");
        while (command != "quit") {
            std::cout << Colors::FG_GREEN << ">> " << Colors::FG_CYAN;

            try {
                std::getline(std::cin, command);
                std::cout << Colors::RESET;   // reset color
                cmd_history->add(command);    // save command
                run_command(panda, command);  // perform task

            } catch (const PCEXC::PandaControllerException &e) {
                ROS_ERROR_STREAM(PCEXC::get_err_msg(NAME, e.what()));
            }

            std::cout << Colors::FG_GREEN
                      << "-------------------------------------\n"
                      << Colors::RESET;
        }
    } catch (const PCEXC::PandaControllerException &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}



//#############################################################################
// PRIVATE FUNCTIONS IMPLEMENTATIONS ##########################################
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

            ROS_STRONG_INFO(config::FG, config::BG, "SELECTED QUIT");

            // CASE HELP
        } else if (cmd[0] == "help") {
            if ((cmd.size() != 1))
                throw std::invalid_argument(command);

            ROS_STRONG_INFO(config::FG, config::BG, "SELECTED HELP");
            const auto color = Colors::FG_YELLOW_BRIGHT;
            ROS_INFO_STREAM(
                Colors::BOLD_INTENSITY
                << "\nLeggends:\n"
                << Colors::INTENSITY_OFF << " - [] indicates a parameter.\n"
                << " - () indicates a optional parameter with default "
                   "value in launch file.\n"
                << Colors::BOLD_INTENSITY << "Commands available:\n"
                << Colors::INTENSITY_OFF

                << color << " - quit: " << Colors::RESET
                << "to close the node.\n"

                << color << " - help: " << Colors::RESET
                << "to see the list of commands.\n"

                << color << " - info: " << Colors::RESET
                << "to print robot info.\n"

                << color << " - scene [name:str]: " << Colors::RESET
                << "to load specified scene.\n"

                << color << " - scene reset: " << Colors::RESET
                << "to reset scene.\n"

                << color << " - speed arm [speed:float]: " << Colors::RESET
                << "to set the arm speed value.\n"

                << color << " - eef set [name:str]: " << Colors::RESET
                << "to set end effector link name.\n"

                << color << " - eef get: " << Colors::RESET
                << "to get end effector link name.\n"

                << color << " - speed gripper [speed:float]: " << Colors::RESET
                << "to set the gripper speed value.\n"

                << color << " - save [(name:str)]: " << Colors::RESET
                << "to save the current pose with the specified name\n"

                << color
                << " - save eef [(eef:str), (name:str)]: " << Colors::RESET
                << "to save the current pose with the specified name. You must "
                   "select end effector link name.\n"

                << color
                << " - move joint [joint:int, val:double]: " << Colors::RESET
                << "to move the specified joint by an integer of the specified "
                   "degree.\n"

                << color << " - move offset [x:double y:double z:double]: "
                << Colors::RESET
                << "to move the arm along the x,y,z specified directions in "
                   "meters.\n"

                << color << " - move pose [name:str]: " << Colors::RESET
                << "to move the arm on the specified pose saved in database.\n"

                << color << " - move gripper [width:double]: " << Colors::RESET
                << "to move the gripper fingers with the specified speed on "
                   "the specified width from center.\n"

                << color << " - homing arm: " << Colors::RESET
                << "to perform the homing of the arm.\n"

                << color << " - homing gripper: " << Colors::RESET
                << "to perform the homing of the gripper.\n"

                << color
                << " - grasp [width:double (force:double epsilon_inner:double "
                   "epsilon_outer:double)]: "
                << Colors::RESET << "to perform the grasping.");

            // CASE INFO
        } else if (cmd[0] == "info") {
            if ((cmd.size() != 1))
                throw std::invalid_argument(command);

            ROS_STRONG_INFO(config::FG, config::BG, "SELECTED INFO");
            ROS_INFO_STREAM(panda.getLinkNames());
            ROS_INFO_STREAM(panda.getJointNames());

            // CASE SCENE
        } else if (cmd[0] == "scene") {
            if ((cmd.size() != 2))
                throw std::invalid_argument(command);

            ROS_STRONG_INFO(config::FG, config::BG, "SELECTED LOAD");
            std::string scene = cmd[1];
            if (scene == "reset") {
                ROS_INFO_STREAM("Reset scene");
                panda.resetScene();
            } else {
                ROS_INFO_STREAM("Load scene: " << scene);
                panda.setScene(data_manager::get_scene(scene));
            }

            // CASE SPEED
        } else if (cmd[0] == "speed") {
            if (cmd.size() < 2)
                throw std::invalid_argument(command);

            if (cmd[1] == "arm") {
                if ((cmd.size() != 3) || !is_number(cmd[2]))
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(config::FG, config::BG, "SELECTED ARM SPEED");
                float speed = std::stof(cmd[2]);
                ROS_INFO_STREAM("Set arm speed to: " << speed);
                panda.setArmSpeed(speed);

            } else if (cmd[1] == "gripper") {
                if ((cmd.size() != 3) || !is_number(cmd[2]))
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(config::FG, config::BG,
                                "SELECTED GRIPPER SPEED");
                float speed = std::stof(cmd[2]);
                ROS_INFO_STREAM("Set gripper speed to: " << speed);
                panda.setGripperSpeed(speed);
            } else {
                throw std::invalid_argument(command);
            }

            // CASE EEF
        } else if (cmd[0] == "eef") {
            if (cmd.size() > 3 || cmd.size() < 2 ||
                (cmd.size() == 2 && cmd[1] != "get") ||
                (cmd.size() == 3 && cmd[1] != "set"))
                throw std::invalid_argument(command);

            ROS_STRONG_INFO(config::FG, config::BG, "SELECTED EEF");

            if (cmd.size() == 2) {
                ROS_INFO_STREAM(
                    "Get end effector link: " << panda.getEndEffectorLink());

            } else {
                ROS_INFO_STREAM("Set end effector link to: " << cmd[2]);
                panda.setEndEffectorLink(cmd[2]);
            }

            // CASE SAVE
        } else if (cmd[0] == "save") {
            if (cmd.size() > 4 || (cmd.size() > 2 && cmd[1] != "eef") ||
                (cmd.size() == 2 && cmd[1] == "eef"))
                throw std::invalid_argument(command);

            ROS_STRONG_INFO(config::FG, config::BG,
                            "SELECTED SAVE CURRENT POSE");

            if (cmd.size() > 2 && cmd[1] == "eef") {
                auto pose = panda.getCurrentPose(cmd[2]);
                ROS_INFO_STREAM("Save pose:\n" << pose);
                if (cmd.size() == 4) {
                    data_manager::save_pose(pose, cmd[3]);
                } else {
                    data_manager::save_pose(pose);
                }

            } else {
                auto pose = panda.getCurrentPose();
                ROS_INFO_STREAM("Save pose:\n" << pose);
                if (cmd.size() == 2) {
                    data_manager::save_pose(pose, cmd[1]);
                } else {
                    data_manager::save_pose(pose);
                }
            }


            // CASE MOVE
        } else if (cmd[0] == "move") {
            if (cmd.size() < 2)
                throw std::invalid_argument(command);

            if (cmd[1] == "joint") {
                if ((cmd.size() != 4) || !is_number(cmd[2]) ||
                    !is_number(cmd[3]))
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(config::FG, config::BG, "SELECTED MOVE JOINT");
                int joint = std::stoi(cmd[2]);
                double val = std::stod(cmd[3]);
                ROS_INFO_STREAM("Move joint " << joint << " of " << val
                                              << " deg\n");
                panda.moveJointDeg(joint, val, config::JOINT_ADJUST_IN_BOUNDS);

            } else if (cmd[1] == "offset") {
                if ((cmd.size() != 5) || !is_number(cmd[2]) ||
                    !is_number(cmd[3]) || !is_number(cmd[4]))
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(config::FG, config::BG, "SELECTED MOVE OFFSET");
                double x = std::stod(cmd[2]);
                double y = std::stod(cmd[3]);
                double z = std::stod(cmd[4]);
                auto target_pose = panda.getCurrentPose();
                target_pose.position.x += x;
                target_pose.position.y += y;
                target_pose.position.z += z;
                ROS_INFO_STREAM("Move to pose:\n" << target_pose);
                panda.cartesianMovement(target_pose, config::CARTESIAN_STEP,
                                        config::CARTESIAN_JUMP);

            } else if (cmd[1] == "pose") {
                if (cmd.size() != 3)
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(config::FG, config::BG,
                                "SELECTED MOVE TO POSE");
                ROS_INFO_STREAM("Get pose: " << cmd[2]);
                auto target_pose = data_manager::get_pose(cmd[2]);
                ROS_INFO_STREAM("Move to pose:\n" << target_pose);
                panda.moveToPose(target_pose);

            } else if (cmd[1] == "gripper") {
                if (cmd.size() != 3 || !is_number(cmd[2]))
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(config::FG, config::BG,
                                "SELECTED GRIPPER MOVE");
                double width = std::stod(cmd[2]);
                ROS_INFO_STREAM("Move gripper to: " << width);
                panda.gripperMove(width);

            } else {
                throw std::invalid_argument(command);
            }

            // CASE HOMING
        } else if (cmd[0] == "homing") {
            if (cmd.size() != 2)
                throw std::invalid_argument(command);

            if (cmd[1] == "arm") {
                ROS_STRONG_INFO(config::FG, config::BG, "SELECTED ARM HOMING");
                panda.moveToReadyPose();

            } else if (cmd[1] == "gripper") {
                ROS_STRONG_INFO(config::FG, config::BG,
                                "SELECTED GRIPPER HOMING");
                panda.gripperHoming();

            } else {
                throw std::invalid_argument(command);
            }

            // CASE GRASP
        } else if (cmd[0] == "grasp") {
            if ((cmd.size() != 2 && cmd.size() != 5) || !is_number(cmd[1]) ||
                (cmd.size() == 5 && !(is_number(cmd[2]) && is_number(cmd[3]) &&
                                      is_number(cmd[4]))))
                throw std::invalid_argument(command);

            ROS_STRONG_INFO(config::FG, config::BG, "SELECTED GRASP");
            double width = std::stod(cmd[1]);
            double force, epsilon_inner, epsilon_outer;
            if (cmd.size() == 5) {
                force = std::stof(cmd[2]);
                epsilon_inner = std::stof(cmd[3]);
                epsilon_outer = std::stof(cmd[4]);
            } else {
                force = config::GRASP_FORCE;
                epsilon_inner = config::GRASP_EPSILON_INNER;
                epsilon_outer = config::GRASP_EPSILON_OUTER;
            }
            ROS_INFO_STREAM("With: "
                            << "\n - width: " << width << "\n - force: "
                            << force << "\n - epsilon_inner: " << epsilon_inner
                            << "\n - epsilon_outer: " << epsilon_outer);
            panda.gripperGrasp(width, force, epsilon_inner, epsilon_outer);

        } else {
            throw std::invalid_argument(command);
        }

    } catch (const std::invalid_argument &e) {
        ROS_WARN_STREAM("invalid command: " << e.what());
    }
}


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



//#############################################################################
// PRIVATE CLASSES IMPLEMENTATIONS ############################################
History::History(const int &SIZE) {
    max_size_ = SIZE;
}

void History::setSize(const int &SIZE) {
    max_size_ = SIZE;
}

void History::add(const std::string &COMMAND) {
    if (history_.size() >= max_size_)
        history_.pop_back();      // Remove oldest value
    history_.push_back(COMMAND);  // Add value
    cursor_ = history_.size();    // Reset cursor
}

std::string History::prev() {
    if (cursor_ == 0) {
        cursor_ = -1;
        return "";
    }
    if (cursor_ > 0) {
        cursor_--;
        return history_[cursor_];
    }
    return "history_err";
}

std::string History::next() {
    if (cursor_ == static_cast<size_t>(history_.size() - 1)) {
        cursor_ = history_.size();
        return "";
    }
    if (cursor_ < static_cast<size_t>(history_.size() - 1)) {
        cursor_++;
        return history_[cursor_];
    }
    return "history_err";
}