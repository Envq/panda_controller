// PANDA CONTROLLER
#include "colors.hpp"                       //ROS_STRONG_INFO
#include "exceptions.hpp"                   //PCEXC
#include "panda_controller/teleop_panda.h"  //MSG

// ROS
#include <ros/ros.h>

// C
#include <termios.h>
#include <unistd.h>



//#############################################################################
// CONFIGS ####################################################################
namespace config {
const auto FG = Colors::FG_BLUE;
const auto BG = Colors::BG_BLACK;

const int QUIT = 27;  // esc
const int HELP = 'h';
const int MODE = 'm';
const int X_POS = 'w';
const int X_NEG = 's';
const int Y_POS = 'a';
const int Y_NEG = 'd';
const int Z_POS = 'i';
const int Z_NEG = 'k';
const int INCREASE_POSITION = 'l';
const int DECREASE_POSITION = 'j';
const int ROLL_POS = 'd';              // '6';
const int ROLL_NEG = 'a';              //'4';
const int PITCH_POS = 'w';             //'8';
const int PITCH_NEG = 's';             //'5';
const int YAW_POS = 'q';               //'7';
const int YAW_NEG = 'e';               //'9';
const int INCREASE_ORIENTATION = 'l';  //'3';
const int DECREASE_ORIENTATION = 'j';  //'1';
const int GRIPPER_HOMING = 'o';
const int ARM_HOMING = 'o';
const int GRIPPER_GRASP = 's';
const int GRIPPER_WIDTH_POS = 'd';
const int GRIPPER_WIDTH_NEG = 'a';
const int INCREASE_GRIPPER_WIDTH = 'l';
const int DECREASE_GRIPPER_WIDTH = 'j';
}  // namespace config



//#############################################################################
// PRIVATE FUNCTIONS ##########################################################
int getch();



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
    ROS_STRONG_INFO(config::FG, config::BG, "START NODE: ", NAME);


    // Extract the parameters
    double START_DELTA_POSITION, START_DELTA_ORIENTATION,
        START_DELTA_GRIPPER_WIDTH, RESOLUTION_POSITION, RESOLUTION_ORIENTATION,
        RESOLUTION_GRIPPER_WIDTH, FREQUENCY;
    if (!(node.getParam("frequency", FREQUENCY) &&
          node.getParam("start_delta_position", START_DELTA_POSITION) &&
          node.getParam("start_delta_orientation", START_DELTA_ORIENTATION) &&
          node.getParam("start_delta_gripper_width",
                        START_DELTA_GRIPPER_WIDTH) &&
          node.getParam("resolution_position", RESOLUTION_POSITION) &&
          node.getParam("resolution_orientation", RESOLUTION_ORIENTATION) &&
          node.getParam("resolution_gripper_width",
                        RESOLUTION_GRIPPER_WIDTH))) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Create publisher
    ROS_STRONG_INFO(config::FG, config::BG, "PUBLICATION TO THE TOPIC: teleop");
    ros::Publisher pub = node.advertise<panda_controller::teleop_panda>(
        "/panda_controller/teleop", 1000);


    // Task
    ros::Rate loop_rate(FREQUENCY);
    double delta_position = START_DELTA_POSITION;
    double delta_orientation = START_DELTA_ORIENTATION;
    double delta_gripper_width = START_DELTA_GRIPPER_WIDTH;
    int command;
    int mode = 0;
    panda_controller::teleop_panda msg;
    ROS_INFO_STREAM(
        "START INFO:"
        << "\n- Mode: "
        << ((mode == 0) ? "position" : (mode == 1) ? "orientation" : "gripper")
        << "\n- Delta position:      " << delta_position << " meters"
        << "\n- Delta orientation:   " << delta_orientation << " degrees"
        << "\n- Delta gripper width: " << delta_gripper_width << " meters");

    while (ros::ok()) {
        // Get command
        command = getch();

        // Reset msg
        msg.x = 0.0;
        msg.y = 0.0;
        msg.z = 0.0;
        msg.roll = 0.0;
        msg.pitch = 0.0;
        msg.yaw = 0.0;
        msg.gripper_width = 0.0;
        msg.gripper_grasp = 0.0;
        msg.arm_homing = false;
        msg.gripper_homing = false;

        // QUIT case
        if (command == config::QUIT) {
            ROS_STRONG_INFO(config::FG, config::BG, "QUIT");
            break;

            // HELP case
        } else if (command == config::HELP) {
            std::cout << "#########################"
                      << "\nQUIT:                 ESC"
                      << "\nHELP:                   "
                      << static_cast<char>(config::HELP)
                      << "\nMODE:                   "
                      << static_cast<char>(config::MODE)
                      << "\n#########################"
                      << "\nARM HOMING:             "
                      << static_cast<char>(config::ARM_HOMING)
                      << "\nX POS:                  "
                      << static_cast<char>(config::X_POS)
                      << "\nX NEG:                  "
                      << static_cast<char>(config::X_NEG)
                      << "\nY NEG:                  "
                      << static_cast<char>(config::Y_NEG)
                      << "\nY POS:                  "
                      << static_cast<char>(config::Y_POS)
                      << "\nZ POS:                  "
                      << static_cast<char>(config::Z_POS)
                      << "\nZ NEG:                  "
                      << static_cast<char>(config::Z_NEG)
                      << "\nINCREASE POSITION:      "
                      << static_cast<char>(config::INCREASE_POSITION)
                      << "\nDECREASE POSITION:      "
                      << static_cast<char>(config::DECREASE_POSITION)
                      << "\n#########################"
                      << "\nROLL POS:               "
                      << static_cast<char>(config::ROLL_POS)
                      << "\nROLL NEG:               "
                      << static_cast<char>(config::ROLL_NEG)
                      << "\nPITCH POS:              "
                      << static_cast<char>(config::PITCH_POS)
                      << "\nPITCH NEG:              "
                      << static_cast<char>(config::PITCH_NEG)
                      << "\nYAW POS:                "
                      << static_cast<char>(config::YAW_POS)
                      << "\nYAW NEG:                "
                      << static_cast<char>(config::YAW_NEG)
                      << "\nINCREASE ORIENTATION:   "
                      << static_cast<char>(config::INCREASE_ORIENTATION)
                      << "\nDECREASE ORIENTATION:   "
                      << static_cast<char>(config::DECREASE_ORIENTATION)
                      << "\n#########################"
                      << "\nGRIPPER HOMING:         "
                      << static_cast<char>(config::GRIPPER_HOMING)
                      << "\nGRIPPER GRASP           "
                      << static_cast<char>(config::GRIPPER_GRASP)
                      << "\nGRIPPER WIDTH POS:      "
                      << static_cast<char>(config::GRIPPER_WIDTH_POS)
                      << "\nGRIPPER WIDTH NEG:      "
                      << static_cast<char>(config::GRIPPER_WIDTH_NEG)
                      << "\nINCREASE GRIPPER WIDTH: "
                      << static_cast<char>(config::INCREASE_GRIPPER_WIDTH)
                      << "\nDECREASE GRIPPER WIDTH: "
                      << static_cast<char>(config::DECREASE_GRIPPER_WIDTH)
                      << "\n#########################" << std::endl;

        } else if (command == config::MODE) {
            mode = (mode + 1) % 3;
            std::cout << "MODE: "
                      << ((mode == 0) ? "position"
                                      : (mode == 1) ? "orientation" : "gripper")
                      << std::endl;

        } else if (mode == 0) {
            // DELTA POSITION cases
            if (command == config::INCREASE_POSITION) {
                delta_position += RESOLUTION_POSITION;
                std::cout << "Delta position: " << delta_position << " meters"
                          << std::endl;

            } else if (command == config::DECREASE_POSITION) {
                if ((delta_position - RESOLUTION_POSITION) > 0)
                    delta_position -= RESOLUTION_POSITION;
                std::cout << "Delta position: " << delta_position << " meters"
                          << std::endl;

            } else {
                // POSITION cases
                if (command == config::ARM_HOMING) {
                    msg.arm_homing = true;

                } else if (command == config::X_POS) {
                    msg.x += delta_position;

                } else if (command == config::X_NEG) {
                    msg.x -= delta_position;

                } else if (command == config::Y_NEG) {
                    msg.y -= delta_position;

                } else if (command == config::Y_POS) {
                    msg.y += delta_position;

                } else if (command == config::Z_POS) {
                    msg.z += delta_position;

                } else if (command == config::Z_NEG) {
                    msg.z -= delta_position;

                } else {
                    continue;  // Not publish
                }
                pub.publish(msg);
                ros::spinOnce();
            }

        } else if (mode == 1) {
            // DELTA ORIENTATION cases
            if (command == config::INCREASE_ORIENTATION) {
                delta_orientation += RESOLUTION_ORIENTATION;
                std::cout << "Delta orientation: " << delta_orientation
                          << " degrees" << std::endl;

            } else if (command == config::DECREASE_ORIENTATION) {
                if ((delta_orientation - RESOLUTION_ORIENTATION) > 0)
                    delta_orientation -= RESOLUTION_ORIENTATION;
                std::cout << "Delta orientation: " << delta_orientation
                          << " degrees" << std::endl;

            } else {
                // ORIENTATION cases
                if (command == config::ARM_HOMING) {
                    msg.arm_homing = true;

                } else if (command == config::ROLL_POS) {
                    msg.roll += delta_orientation;

                } else if (command == config::ROLL_NEG) {
                    msg.roll -= delta_orientation;

                } else if (command == config::PITCH_POS) {
                    msg.pitch += delta_orientation;

                } else if (command == config::PITCH_NEG) {
                    msg.pitch -= delta_orientation;

                } else if (command == config::YAW_POS) {
                    msg.yaw += delta_orientation;

                } else if (command == config::YAW_NEG) {
                    msg.yaw -= delta_orientation;

                } else {
                    continue;  // Not publish
                }
                pub.publish(msg);
                ros::spinOnce();
            }

        } else if (mode == 2) {
            // DELTA GRIPPER WIDTH cases
            if (command == config::INCREASE_GRIPPER_WIDTH) {
                delta_gripper_width += RESOLUTION_GRIPPER_WIDTH;
                std::cout << "Delta gripper width: " << delta_gripper_width
                          << " meters" << std::endl;

            } else if (command == config::DECREASE_GRIPPER_WIDTH) {
                if ((delta_gripper_width - RESOLUTION_GRIPPER_WIDTH) > 0)
                    delta_gripper_width -= RESOLUTION_GRIPPER_WIDTH;
                std::cout << "Delta gripper width: " << delta_gripper_width
                          << " meters" << std::endl;

            } else {
                // GRIPPER WIDTH cases
                if (command == config::GRIPPER_GRASP) {
                    msg.gripper_grasp = delta_gripper_width;

                } else if (command == config::GRIPPER_HOMING) {
                    msg.gripper_homing = true;

                } else if (command == config::GRIPPER_WIDTH_POS) {
                    msg.gripper_width += delta_gripper_width;

                } else if (command == config::GRIPPER_WIDTH_NEG) {
                    msg.gripper_width -= delta_gripper_width;

                } else {
                    continue;  // Not publish
                }
                pub.publish(msg);
                ros::spinOnce();
            }
        }
        loop_rate.sleep();  // sync with loop_rate
    }


    // Finish
    ros::shutdown();
    return 0;
}



//#############################################################################
// PRIVATE FUNCTIONS IMPLEMENTATIONS ##########################################
int getch() {
    int ch;
    struct termios oldt;
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt); /*store old settings */
    newt = oldt;                    /* copy old settings to new settings */
    newt.c_lflag &=
        ~(ICANON | ECHO); /* make one change to old settings in new settings */
    tcsetattr(STDIN_FILENO, TCSANOW,
              &newt); /*apply the new settings immediatly */
    ch = getchar();   /* standard getchar call */
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*reapply the old settings */
    return ch;                               /*return received char */
}