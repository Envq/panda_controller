#pragma once

// PANDA CONTROLLER
#include "exceptions.hpp"  //PCEXC

// C++
#include <fstream>
#include <jsoncpp/json/json.h>
#include <vector>

// ROS
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <shape_msgs/SolidPrimitive.h>

// MOVEIT
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>



//#############################################################################
// NAMESPACE ##################################################################
namespace data_manager {



//#############################################################################
// CONSTANTS ###################################################################
const std::string PACKAGE_NAME = "panda_controller";
const std::string FOLDER_NAME = "/data/";
const std::string POSES_RELATIVE = FOLDER_NAME + "poses.json";
const std::string SCENES_RELATIVE = FOLDER_NAME + "scenes.json";



//#############################################################################
// FUNCTIONS INTERFACE ########################################################

// Save the pose with name
void save_pose(const geometry_msgs::Pose &POSE,
               const std::string &NAME = "current");


// Get the pose indicated
geometry_msgs::Pose get_pose(const std::string &NAME);


// Initialize the scene indicated
moveit_msgs::PlanningScene get_scene(const std::string &NAME);


}  // namespace data_manager