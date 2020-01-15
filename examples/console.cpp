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
// DEFAULT VALUES ##############################################################
const int HISTORY_MAX_SIZE = 20;
const auto FG_COLOR = Colors::FG_BLUE;
const auto BG_COLOR = Colors::BG_BLACK;



//#############################################################################
// GLOBAL VALUES ##############################################################
float GRIPPER_FORCE_DFLT, GRIPPER_EPSILON_INNER_DFLT,
    GRIPPER_EPSILON_OUTER_DFLT;



//#############################################################################
// PRIVATE FUNCTIONS ##########################################################
// Check if a string is a number
bool is_number(const std::string &str);

// Perform parsing of readed command and run it
void run_command(robot::Panda &panda, const std::string &command);



//#############################################################################
// PRIVATE CLASSES ############################################################
// Manages history of commands
class History {
  private:
    int max_size_;
    std::vector<std::string> history_;
    int cursor_;

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
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "START NODE: ", NAME);


    // Extract the parameters
    if (!(node.getParam("gripper_force", GRIPPER_FORCE_DFLT) &&
          node.getParam("gripper_epsilon_inner", GRIPPER_EPSILON_INNER_DFLT) &&
          node.getParam("gripper_epsilon_outer", GRIPPER_EPSILON_OUTER_DFLT))) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Task
    try {
        // Create class to manage the Panda arm
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "PANDA CONTROLLER INITIALIZATION");
        auto panda = robot::Panda();

        // Read command and performe task
        int scan;
        std::string command;
        boost::shared_ptr<History> cmd_history(
            new History(HISTORY_MAX_SIZE));  // Create history
        ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "INSERT COMMANDS:");
        while (command != "quit") {
            std::cout << Colors::FG_GREEN << ">> " << Colors::FG_CYAN;

            command = "";  // Reset command
            try {
                // while ((scan = getchar()) != 10) {
                //     command += scan;
                // }
                std::getline(std::cin, command);
                std::cout << Colors::RESET;   // reset color
                cmd_history->add(command);    // save command
                run_command(panda, command);  // perform task

            } catch (const PCEXC::panda_error &e) {
                ROS_ERROR_STREAM(PCEXC::get_err_msg(NAME, e.what()));

            } catch (const PCEXC::data_manager_error &e) {
                ROS_ERROR_STREAM(PCEXC::get_err_msg(NAME, e.what()));
            }

            std::cout << Colors::FG_GREEN
                      << "-------------------------------------\n"
                      << Colors::RESET;
        }
    } catch (const PCEXC::panda_error &e) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, e.what()));
    }


    // Finish
    ros::shutdown();
    return 0;
}



//#############################################################################
// PRIVATE FUNCTIONS IMPLEMENTATIONS ##########################################
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

            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED QUIT");

            // CASE HELP
        } else if (cmd[0] == "help") {
            if ((cmd.size() != 1))
                throw std::invalid_argument(command);

            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED HELP");
            ROS_INFO_STREAM(
                Colors::BOLD_INTENSITY
                << "\nLeggends:\n"
                << Colors::INTENSITY_OFF << " - [] indicates a parameter.\n"
                << " - () indicates a optional parameter with default "
                   "value in launch file.\n"
                << Colors::BOLD_INTENSITY << "Commands available:\n"
                << Colors::INTENSITY_OFF
                << " - help: to see the list of commands.\n"
                << " - quit: to close the node.\n"
                << " - scene [name]: to load specified scene.\n"
                << " - scene reset to reset scene.\n"
                << " - speed arm [value]: to set the arm speed value.\n"
                << " - speed gripper [value]: to set the gripper speed value.\n"
                << " - save [(name)]: to save the current pose with the "
                   "specified name.\n"
                << " - move offset [x y z]: to move the arm along the "
                   "x,y,z specified directions in meters.\n"
                << " - move pose [name]: to move the arm on the specified "
                   "pose saved in database.\n"
                << " - move gripper [width (speed)]: to move the gripper "
                   "fingers with the specified speed on the specified width "
                   "from center.\n"
                << " - homing arm: to perform the homing of the arm.\n"
                << " - homing gripper: to perform the homing of the gripper.\n"
                << " - grasp [width (speed force epsilon_inner "
                   "epsilon_outer)]: to perform the grasping.");

            // CASE SCENE
        } else if (cmd[0] == "scene") {
            if ((cmd.size() != 2))
                throw std::invalid_argument(command);

            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED LOAD");
            std::string scene = cmd[1];
            if (scene == "reset") {
                ROS_INFO_STREAM("Reset scene");
                panda.resetScene();
            } else {
                ROS_INFO_STREAM("Load scene:" << scene);
                panda.setScene(data_manager::get_scene(scene));
            }

            // CASE SPEED
        } else if (cmd[0] == "speed") {
            if (cmd.size() < 2)
                throw std::invalid_argument(command);

            if (cmd[1] == "arm") {
                if ((cmd.size() != 3) || !is_number(cmd[2]))
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED ARM SPEED");
                float speed = std::stof(cmd[2]);
                ROS_INFO_STREAM("Set arm speed to:" << speed);
                panda.setArmSpeed(speed);

            } else if (cmd[1] == "gripper") {
                if ((cmd.size() != 3) || !is_number(cmd[2]))
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED GRIPPER SPEED");
                float speed = std::stof(cmd[2]);
                ROS_INFO_STREAM("Set gripper speed to:" << speed);
                panda.setGripperSpeed(speed);
            } else {
                throw std::invalid_argument(command);
            }

            // CASE SAVE
        } else if (cmd[0] == "save") {
            if (cmd.size() > 2)
                throw std::invalid_argument(command);

            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED SAVE CURRENT POSE");
            auto pose = panda.getCurrentPose();
            ROS_INFO_STREAM("Save pose:\n" << pose);
            if (cmd.size() == 2) {
                std::string pose_name = cmd[1];
                data_manager::save_pose(pose, pose_name);
            } else {
                data_manager::save_pose(pose);
            }

            // CASE MOVE
        } else if (cmd[0] == "move") {
            if (cmd.size() < 2)
                throw std::invalid_argument(command);

            if (cmd[1] == "offset") {
                if ((cmd.size() != 5) || !is_number(cmd[2]) ||
                    !is_number(cmd[3]) || !is_number(cmd[4]))
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED MOVE OFFSET");
                float x = std::stof(cmd[2]);
                float y = std::stof(cmd[3]);
                float z = std::stof(cmd[4]);
                auto target_pose = panda.getCurrentPose();
                target_pose.position.x += x;
                target_pose.position.y += y;
                target_pose.position.z += z;
                ROS_INFO_STREAM("Move to pose:\n" << target_pose);
                panda.cartesianMovement(target_pose);

            } else if (cmd[1] == "pose") {
                if (cmd.size() != 3)
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED MOVE TO POSE");
                ROS_INFO_STREAM("Get pose: " << cmd[2]);
                auto target_pose = data_manager::get_pose(cmd[2]);
                ROS_INFO_STREAM("Move to pose:\n" << target_pose);
                panda.moveToPosition(target_pose);

            } else if (cmd[1] == "gripper") {
                if (cmd.size() != 3 || !is_number(cmd[2]))
                    throw std::invalid_argument(command);

                ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED GRIPPER MOVE");
                float width = std::stof(cmd[2]);
                float speed = std::stof(cmd[3]);
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
                ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED ARM HOMING");
                auto target_pose = data_manager::get_pose("ready");
                ROS_INFO_STREAM("Move to pose:\n" << target_pose);
                panda.moveToPosition(target_pose);

            } else if (cmd[1] == "gripper") {
                ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED GRIPPER HOMING");
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

            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "SELECTED GRASP");
            float width = std::stof(cmd[1]);
            float speed, force, epsilon_inner, epsilon_outer;
            if (cmd.size() == 5) {
                force = std::stof(cmd[2]);
                epsilon_inner = std::stof(cmd[3]);
                epsilon_outer = std::stof(cmd[4]);
            } else {
                force = GRIPPER_FORCE_DFLT;
                epsilon_inner = GRIPPER_EPSILON_INNER_DFLT;
                epsilon_outer = GRIPPER_EPSILON_OUTER_DFLT;
            }
            ROS_INFO_STREAM("With:"
                            << "\n - width: " << width << "\n - speed: "
                            << speed << "\n - force: " << force
                            << "\n - epsilon_inner: " << epsilon_inner
                            << "\n - epsilon_outer: " << epsilon_outer);
            panda.gripperGrasp(width, force, epsilon_inner, epsilon_outer);

        } else {
            throw std::invalid_argument(command);
        }

    } catch (const std::invalid_argument &e) {
        ROS_WARN_STREAM("invalid command: " << e.what());
    }
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
    if (cursor_ == static_cast<int>(history_.size() - 1)) {
        cursor_ == history_.size();
        return "";
    }
    if (cursor_ < static_cast<int>(history_.size() - 1)) {
        cursor_++;
        return history_[cursor_];
    }
    return "history_err";
}