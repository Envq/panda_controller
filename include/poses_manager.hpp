#pragma once

// MY LIBS
#include "my_exceptions.hpp"

// C++
#include <fstream>
#include <jsoncpp/json/json.h>

// ROS
#include <geometry_msgs/Pose.h>
#include <ros/package.h>



//#############################################################################
// FUNCTIONS
namespace poses_manager {

void save_pose(const std::string &NAME, const geometry_msgs::Pose &pose);

geometry_msgs::Pose get_pose(const std::string &NAME);

}  // namespace poses_manager