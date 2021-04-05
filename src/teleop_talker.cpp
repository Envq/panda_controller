// ROS
#include <ros/ros.h>

// C++
#include <iostream>
#include <termios.h>
#include <unistd.h>

// BOOST
#include <boost/lexical_cast.hpp>

// Custom
#include "panda_controller/exceptions.hpp"
#include "panda_controller/panda.hpp"
#include "panda_controller/panda_teleop.h"  // msg
#include "utils/colors.hpp"



// USING NAMESPACE ============================================================
using namespace panda_controller;



// CONFIGS ====================================================================
auto FG_COLOR = Colors::FG_BLUE;
auto BG_COLOR = Colors::BG_DEFAULT;
auto CMD1_COLOR = Colors::FG_GREEN;
auto INFO_COLOR = Colors::FG_CYAN;



// FUNCTIONS ==================================================================
// Returns the pressed key
int getch() {
    int ch;
    struct termios oldt;
    struct termios newt;
    tcgetattr(STDIN_FILENO, &oldt);  // store old settings
    newt = oldt;                     // copy old settings to new settings
    newt.c_lflag &=                  //
        ~(ICANON | ECHO);  // make one change to old settings in new settings
    tcsetattr(STDIN_FILENO, TCSANOW,  //
              &newt);                 // apply the new settings immediatly
    ch = getchar();                   // standard getchar call
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // reapply the old settings
    return ch;                                // return received char
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
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "START NODE: ", CURRENT_FILE_NAME);


    // DEBUG: check key pressed (27 == ESC)
    // int debug_cmd;
    // while (debug_cmd != 27) {
    //     debug_cmd = getch();
    //     ROS_WARN_STREAM("cmd: " << debug_cmd);
    //     ros::Duration(0.1).sleep();
    // }
    // ros::shutdown();
    // return 0;


    // Extract the parameters
    double FREQUENCY;
    double START_DELTA_POSITION, START_DELTA_ORIENTATION, START_DELTA_GRIPPER;
    double GRANULARITY_POSITION, GRANULARITY_ORIENTATION, GRANULARITY_GRIPPER;
    int QUIT, HELP, MODE, READY;
    int X_NEG, X_POS, Y_NEG, Y_POS, Z_NEG, Z_POS, DEC_POS, INC_POS;
    int ROLL_NEG, ROLL_POS, PITCH_NEG, PITCH_POS, YAW_NEG, YAW_POS, DEC_ORIE,
        INC_ORIE;
    int GR_HOMING, GR_GRASP, GR_W_NEG, GR_W_POS, DEC_GR, INC_GR;
    if (!(node.getParam("frequency", FREQUENCY) &&
          node.getParam("start_delta_position", START_DELTA_POSITION) &&
          node.getParam("start_delta_orientation", START_DELTA_ORIENTATION) &&
          node.getParam("start_delta_gripper", START_DELTA_GRIPPER) &&
          node.getParam("granularity_position", GRANULARITY_POSITION) &&
          node.getParam("granularity_orientation", GRANULARITY_ORIENTATION) &&
          node.getParam("granularity_gripper", GRANULARITY_GRIPPER) &&
          node.getParam("QUIT", QUIT) && node.getParam("HELP", HELP) &&
          node.getParam("MODE", MODE) && node.getParam("X_POS", X_POS) &&
          node.getParam("X_NEG", X_NEG) && node.getParam("Y_POS", Y_POS) &&
          node.getParam("Y_NEG", Y_NEG) && node.getParam("Z_POS", Z_POS) &&
          node.getParam("Z_NEG", Z_NEG) && node.getParam("INC_POS", INC_POS) &&
          node.getParam("DEC_POS", DEC_POS) && node.getParam("READY", READY) &&
          node.getParam("ROLL_POS", ROLL_POS) &&
          node.getParam("ROLL_NEG", ROLL_NEG) &&
          node.getParam("PITCH_POS", PITCH_POS) &&
          node.getParam("PITCH_NEG", PITCH_NEG) &&
          node.getParam("YAW_POS", YAW_POS) &&
          node.getParam("YAW_NEG", YAW_NEG) &&
          node.getParam("INC_ORIE", INC_ORIE) &&
          node.getParam("DEC_ORIE", DEC_ORIE) &&
          node.getParam("GR_HOMING", GR_HOMING) &&
          node.getParam("GR_GRASP", GR_GRASP) &&
          node.getParam("GR_W_POS", GR_W_POS) &&
          node.getParam("GR_W_NEG", GR_W_NEG) &&
          node.getParam("INC_GR", INC_GR) && node.getParam("DEC_GR", DEC_GR))) {
        ROS_FATAL_STREAM(
            get_err_msg(CURRENT_FILE_NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Create publisher
    ROS_FCOL_INFO(FG_COLOR, BG_COLOR,
                  "PUBLICATION TO THE TOPIC: teleoperation");
    ros::Publisher pub = node.advertise<panda_controller::panda_teleop>(
        "/panda_controller/teleoperation", 1000);


    // Task
    ros::Rate loop_rate(FREQUENCY);
    double delta_position = START_DELTA_POSITION;
    double delta_orientation = START_DELTA_ORIENTATION;
    double delta_gripper = START_DELTA_GRIPPER;
    int command;
    int mode = 0;
    panda_controller::panda_teleop msg;
    ROS_INFO_STREAM(
        "START INFO:\n"
        << "- Mode: "
        << ((mode == 0) ? "position" : (mode == 1) ? "orientation" : "gripper")
        << "\n"
        << "- Delta position:      " << delta_position << " meters\n"
        << "- Delta orientation:   " << delta_orientation << " degrees\n"
        << "- Delta gripper width: " << delta_gripper << " meters\n");

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


        if (command == QUIT) {
            ROS_FCOL_INFO(FG_COLOR, BG_COLOR, "QUIT");
            break;


        } else if (command == HELP) {
            std::stringstream help;
            help << Colors::BOLD << FG_COLOR << BG_COLOR;
            help << "Commands available:\n";
            help << Colors::RESET;
            help << CMD1_COLOR;
            help << "[Other]:" << std::endl;
            help << "QUIT:       ESC" << std::endl;
            help << "HELP:       " << static_cast<char>(HELP) << std::endl;
            help << "MODE:       " << static_cast<char>(MODE) << std::endl;
            help << std::endl;

            help << "[Position]:" << std::endl;
            help << "X POS:      " << static_cast<char>(X_POS) << std::endl;
            help << "X NEG:      " << static_cast<char>(X_NEG) << std::endl;
            help << "Y NEG:      " << static_cast<char>(Y_NEG) << std::endl;
            help << "Y POS:      " << static_cast<char>(Y_POS) << std::endl;
            help << "Z POS:      " << static_cast<char>(Z_POS) << std::endl;
            help << "Z NEG:      " << static_cast<char>(Z_NEG) << std::endl;
            help << "[+] DELTA:  " << static_cast<char>(INC_POS) << std::endl;
            help << "[-] DELTA:  " << static_cast<char>(DEC_POS) << std::endl;
            help << "READY:      " << static_cast<char>(READY) << std::endl;
            help << std::endl;

            help << "[Orientation]:" << std::endl;
            help << "ROLL POS:   " << static_cast<char>(ROLL_POS) << std::endl;
            help << "ROLL NEG:   " << static_cast<char>(ROLL_NEG) << std::endl;
            help << "PITCH POS:  " << static_cast<char>(PITCH_POS) << std::endl;
            help << "PITCH NEG:  " << static_cast<char>(PITCH_NEG) << std::endl;
            help << "YAW POS:    " << static_cast<char>(YAW_POS) << std::endl;
            help << "YAW NEG:    " << static_cast<char>(YAW_NEG) << std::endl;
            help << "[+] DELTA:  " << static_cast<char>(INC_ORIE) << std::endl;
            help << "[-] DELTA:  " << static_cast<char>(DEC_ORIE) << std::endl;
            help << "READY:      " << static_cast<char>(READY) << std::endl;
            help << std::endl;

            help << "[Gripper]:" << std::endl;
            help << "HOMING:     " << static_cast<char>(GR_HOMING) << std::endl;
            help << "GRASP       " << static_cast<char>(GR_GRASP) << std::endl;
            help << "WIDTH POS:  " << static_cast<char>(GR_W_POS) << std::endl;
            help << "WIDTH NEG:  " << static_cast<char>(GR_W_NEG) << std::endl;
            help << "[+] DELTA:  " << static_cast<char>(INC_GR) << std::endl;
            help << "[-] DELTA:  " << static_cast<char>(DEC_GR) << std::endl;

            help << Colors::RESET;
            ROS_INFO_STREAM(help.str());


        } else if (command == MODE) {
            mode = (mode + 1) % 3;
            ROS_INFO_STREAM(std::endl
                            << "MODE: "
                            << ((mode == 0)
                                    ? "position"
                                    : (mode == 1) ? "orientation" : "gripper")
                            << std::endl);

        } else if (mode == 0) {
            if (command == INC_POS) {
                delta_position += GRANULARITY_POSITION;
                ROS_INFO_STREAM("Delta position: " << delta_position
                                                   << " meters");

            } else if (command == DEC_POS) {
                if ((delta_position - GRANULARITY_POSITION) > 0)
                    delta_position -= GRANULARITY_POSITION;
                ROS_INFO_STREAM("Delta position: " << delta_position
                                                   << " meters");

            } else {
                if (command == READY) {
                    msg.arm_homing = true;

                } else if (command == X_POS) {
                    msg.x += delta_position;

                } else if (command == X_NEG) {
                    msg.x -= delta_position;

                } else if (command == Y_NEG) {
                    msg.y -= delta_position;

                } else if (command == Y_POS) {
                    msg.y += delta_position;

                } else if (command == Z_POS) {
                    msg.z += delta_position;

                } else if (command == Z_NEG) {
                    msg.z -= delta_position;

                } else {
                    continue;  // Not publish
                }
                pub.publish(msg);
                ros::spinOnce();
            }

        } else if (mode == 1) {
            if (command == INC_ORIE) {
                delta_orientation += GRANULARITY_ORIENTATION;
                ROS_INFO_STREAM("Delta orientation: " << delta_orientation
                                                      << " degrees");

            } else if (command == DEC_ORIE) {
                if ((delta_orientation - GRANULARITY_ORIENTATION) > 0)
                    delta_orientation -= GRANULARITY_ORIENTATION;
                ROS_INFO_STREAM("Delta orientation: " << delta_orientation
                                                      << " degrees");

            } else {
                if (command == READY) {
                    msg.arm_homing = true;

                } else if (command == ROLL_POS) {
                    msg.roll += delta_orientation;

                } else if (command == ROLL_NEG) {
                    msg.roll -= delta_orientation;

                } else if (command == PITCH_POS) {
                    msg.pitch += delta_orientation;

                } else if (command == PITCH_NEG) {
                    msg.pitch -= delta_orientation;

                } else if (command == YAW_POS) {
                    msg.yaw += delta_orientation;

                } else if (command == YAW_NEG) {
                    msg.yaw -= delta_orientation;

                } else {
                    continue;  // Not publish
                }
                pub.publish(msg);
                ros::spinOnce();
            }

        } else if (mode == 2) {
            if (command == INC_GR) {
                delta_gripper += GRANULARITY_GRIPPER;
                ROS_INFO_STREAM("Delta gripper width: " << delta_gripper
                                                        << " meters");

            } else if (command == DEC_GR) {
                if ((delta_gripper - GRANULARITY_GRIPPER) > 0)
                    delta_gripper -= GRANULARITY_GRIPPER;
                ROS_INFO_STREAM("Delta gripper width: " << delta_gripper
                                                        << " meters");

            } else {
                if (command == GR_GRASP) {
                    msg.gripper_grasp = delta_gripper;

                } else if (command == GR_HOMING) {
                    msg.gripper_homing = true;

                } else if (command == GR_W_POS) {
                    msg.gripper_width += delta_gripper;

                } else if (command == GR_W_NEG) {
                    msg.gripper_width -= delta_gripper;

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