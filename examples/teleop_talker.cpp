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
// DEFAULT VALUES ##############################################################
const auto FG_COLOR = Colors::FG_BLUE;
const auto BG_COLOR = Colors::BG_BLACK;
const int QUIT = 'q';
const int HELP = 'h';
const int X_POS = 'w';
const int X_NEG = 's';
const int Y_POS = 'a';
const int Y_NEG = 'd';
const int Z_POS = 'i';
const int Z_NEG = 'k';
const int INCREASE_POSITION = 'l';
const int DECREASE_POSITION = 'j';
const int ROLL_POS = '6';
const int ROLL_NEG = '4';
const int PITCH_POS = '8';
const int PITCH_NEG = '5';
const int YAW_POS = '7';
const int YAW_NEG = '9';
const int INCREASE_ORIENTATION = '3';
const int DECREASE_ORIENTATION = '1';



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
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "START NODE: ", NAME);


    // Extract the parameters
    double FREQUENCY, START_DELTA_POSITION, START_DELTA_ORIENTATION,
        RESOLUTION_POSITION, RESOLUTION_ORIENTATION;
    if (!(node.getParam("frequency", FREQUENCY) &&
          node.getParam("start_delta_position", START_DELTA_POSITION) &&
          node.getParam("start_delta_orientation", START_DELTA_ORIENTATION) &&
          node.getParam("resolution_position", RESOLUTION_POSITION) &&
          node.getParam("resolution_orientation", RESOLUTION_ORIENTATION))) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Create publisher
    ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "PUBLICATION TO THE TOPIC: teleop");
    ros::Publisher pub = node.advertise<panda_controller::teleop_panda>(
        "/panda_controller/teleop", 1000);


    // Task
    ros::Rate loop_rate(FREQUENCY);
    double delta_position = START_DELTA_POSITION;
    double delta_orientation = START_DELTA_ORIENTATION;
    int command;
    ROS_INFO_STREAM("START DELTA:"
                    << "\n- Delta position: " << delta_position << " meters"
                    << "\n- Delta orientation: " << delta_orientation
                    << " degrees");
    while (ros::ok()) {
        panda_controller::teleop_panda msg;
        command = getch();

        // QUIT case
        if (command == QUIT) {
            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "QUIT");
            break;

            // HELP case
        } else if (command == HELP) {
            std::cout << "########################" << std::endl;
            std::cout << "HELP:                  " << static_cast<char>(HELP)
                      << std::endl;
            std::cout << "QUIT:                  " << static_cast<char>(QUIT)
                      << std::endl;
            std::cout << "########################" << std::endl;
            std::cout << "X POS:                 " << static_cast<char>(X_POS)
                      << std::endl;
            std::cout << "X NEG:                 " << static_cast<char>(X_NEG)
                      << std::endl;
            std::cout << "Y NEG:                 " << static_cast<char>(Y_NEG)
                      << std::endl;
            std::cout << "Y POS:                 " << static_cast<char>(Y_POS)
                      << std::endl;
            std::cout << "Z POS:                 " << static_cast<char>(Z_POS)
                      << std::endl;
            std::cout << "Z NEG:                 " << static_cast<char>(Z_NEG)
                      << std::endl;
            std::cout << "INCREASE POSITION:     "
                      << static_cast<char>(INCREASE_POSITION) << std::endl;
            std::cout << "DECREASE POSITION:     "
                      << static_cast<char>(DECREASE_POSITION) << std::endl;
            std::cout << "########################" << std::endl;
            std::cout << "ROLL POS:              "
                      << static_cast<char>(ROLL_POS) << std::endl;
            std::cout << "ROLL NEG:              "
                      << static_cast<char>(ROLL_NEG) << std::endl;
            std::cout << "PITCH POS:             "
                      << static_cast<char>(PITCH_POS) << std::endl;
            std::cout << "PITCH NEG:             "
                      << static_cast<char>(PITCH_NEG) << std::endl;
            std::cout << "YAW POS:               " << static_cast<char>(YAW_POS)
                      << std::endl;
            std::cout << "YAW NEG:               " << static_cast<char>(YAW_NEG)
                      << std::endl;
            std::cout << "INCREASE ORIENTATION:  "
                      << static_cast<char>(INCREASE_ORIENTATION) << std::endl;
            std::cout << "DECREASE ORIENTATION:  "
                      << static_cast<char>(DECREASE_ORIENTATION) << std::endl;
            std::cout << "########################" << std::endl;

            // DELTA POSITION cases
        } else if (command == INCREASE_POSITION) {
            delta_position += RESOLUTION_POSITION;
            std::cout << "Delta position: " << delta_position << " meters"
                      << std::endl;

        } else if (command == DECREASE_POSITION) {
            if ((delta_position - RESOLUTION_POSITION) > 0)
                delta_position -= RESOLUTION_POSITION;
            std::cout << "Delta position: " << delta_position << " meters"
                      << std::endl;

            // DELTA ORIENTATION cases
        } else if (command == INCREASE_ORIENTATION) {
            delta_orientation += RESOLUTION_ORIENTATION;
            std::cout << "Delta orientation: " << delta_orientation
                      << " degrees" << std::endl;

        } else if (command == DECREASE_ORIENTATION) {
            if ((delta_orientation - RESOLUTION_ORIENTATION) > 0)
                delta_orientation -= RESOLUTION_ORIENTATION;
            std::cout << "Delta orientation: " << delta_orientation
                      << " degrees" << std::endl;

        } else {
            // POSITION cases
            if (command == X_POS) {
                msg.y = 0.0;
                msg.x += delta_position;
                msg.z = 0.0;

            } else if (command == X_NEG) {
                msg.y = 0.0;
                msg.x -= delta_position;
                msg.z = 0.0;

            } else if (command == Y_NEG) {
                msg.y -= delta_position;
                msg.x = 0.0;
                msg.z = 0.0;

            } else if (command == Y_POS) {
                msg.y += delta_position;
                msg.x = 0.0;
                msg.z = 0.0;

            } else if (command == Z_POS) {
                msg.y = 0.0;
                msg.x = 0.0;
                msg.z += delta_position;

            } else if (command == Z_NEG) {
                msg.y = 0.0;
                msg.x = 0.0;
                msg.z -= delta_position;


                // ORIENTATION cases
            } else if (command == ROLL_POS) {
                msg.roll += delta_orientation;
                msg.pitch = 0.0;
                msg.yaw = 0.0;

            } else if (command == ROLL_NEG) {
                msg.roll -= delta_orientation;
                msg.pitch = 0.0;
                msg.yaw = 0.0;

            } else if (command == PITCH_POS) {
                msg.roll = 0.0;
                msg.pitch += delta_orientation;
                msg.yaw = 0.0;

            } else if (command == PITCH_NEG) {
                msg.roll = 0.0;
                msg.pitch -= delta_orientation;
                msg.yaw = 0.0;

            } else if (command == YAW_POS) {
                msg.roll = 0.0;
                msg.pitch = 0.0;
                msg.yaw += delta_orientation;

            } else if (command == YAW_NEG) {
                msg.roll = 0.0;
                msg.pitch = 0.0;
                msg.yaw -= delta_orientation;

            } else {
                continue;  // Not publish
            }
            pub.publish(msg);
            ros::spinOnce();
        }
        loop_rate.sleep();  // sync with loop_rate
    }


    // Finish
    ros::shutdown();
    return 0;
}



//#############################################################################
// FUNCTIONS IMPLEMENTATIONS ##################################################
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