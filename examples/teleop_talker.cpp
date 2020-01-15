// PANDA CONTROLLER
#include "colors.hpp"      //ROS_STRONG_INFO
#include "exceptions.hpp"  //PCEXC

// ROS
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>

// C
#include <termios.h>
#include <unistd.h>



//#############################################################################
// DEFAULT VALUES ##############################################################
const auto FG_COLOR = Colors::FG_BLUE;
const auto BG_COLOR = Colors::BG_BLACK;
const int HELP = 'h';
const int ESC = 'q';
const int FRONT = 'w';
const int BACK = 's';
const int RIGHT = 'd';
const int LEFT = 'a';
const int UP = 'i';
const int DOWN = 'k';
const int INCREASE = 'l';
const int DECREASE = 'j';



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
    float FREQUENCY, START_RESOLUTION, DELTA;
    if (!(node.getParam("frequency", FREQUENCY) &&
          node.getParam("start_resolution", START_RESOLUTION) &&
          node.getParam("delta", DELTA))) {
        ROS_FATAL_STREAM(PCEXC::get_err_msg(NAME, "Can't get parameters"));
        ros::shutdown();
        return 0;
    }


    // Create publisher
    ros::Publisher pub = node.advertise<geometry_msgs::Vector3>(
        "/panda_controller/teleop", 1000);


    // Task
    ros::Rate loop_rate(FREQUENCY);
    float resolution = START_RESOLUTION;
    int command;
    while (ros::ok()) {
        geometry_msgs::Vector3 msg;
        switch (command = getch()) {
        case ESC:
            ROS_STRONG_INFO(FG_COLOR, BG_COLOR, "QUIT");
            ros::shutdown();
            break;

        case HELP:
            std::cout << "HELP:     'h'" << std::endl;
            std::cout << "ESC:      'q'" << std::endl;
            std::cout << "FRONT:    'w'" << std::endl;
            std::cout << "BACK:     's'" << std::endl;
            std::cout << "RIGHT:    'd'" << std::endl;
            std::cout << "LEFT:     'a'" << std::endl;
            std::cout << "UP:       'i'" << std::endl;
            std::cout << "DOWN:     'k'" << std::endl;
            std::cout << "INCREASE: 'l'" << std::endl;
            std::cout << "DECREASE: 'j'" << std::endl;
            break;

        case FRONT:
            msg.y = 0.0;
            msg.x += resolution;
            msg.z = 0.0;
            pub.publish(msg);
            ros::spinOnce();
            break;

        case BACK:
            msg.y = 0.0;
            msg.x -= resolution;
            msg.z = 0.0;
            pub.publish(msg);
            ros::spinOnce();
            break;

        case RIGHT:
            msg.y -= resolution;
            msg.x = 0.0;
            msg.z = 0.0;
            pub.publish(msg);
            ros::spinOnce();
            break;

        case LEFT:
            msg.y += resolution;
            msg.x = 0.0;
            msg.z = 0.0;
            pub.publish(msg);
            ros::spinOnce();
            break;

        case UP:
            msg.y = 0.0;
            msg.x = 0.0;
            msg.z += resolution;
            pub.publish(msg);
            ros::spinOnce();
            break;

        case DOWN:
            msg.y = 0.0;
            msg.x = 0.0;
            msg.z -= resolution;
            pub.publish(msg);
            ros::spinOnce();
            break;

        case INCREASE:
            resolution += DELTA;
            std::cout << "Resolution: " << resolution << std::endl;
            break;

        case DECREASE:
            if (resolution - DELTA > 0)
                resolution -= DELTA;
            std::cout << "Resolution: " << resolution << std::endl;
            break;

        default:
            break;
        }
        loop_rate.sleep();  // sync with loop_rate
    }

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