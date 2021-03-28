/**
 * @file data_manager.hpp
 * @author Enrico Sgarbanti
 * @brief This file contains functions for managing data
 * @version 0.1
 * @date 20-02-2020
 *
 * @copyright Copyright (c) 2020 by Enrico Sgarbanti. License GPLv3.
 *
 */
#pragma once

// PANDA CONTROLLER
#include "exceptions.hpp"  //PCEXC

// C++
#include <fstream>
#include <geometric_shapes/shape_operations.h>  //Mesh
#include <jsoncpp/json/json.h>
#include <vector>
#include <yaml-cpp/yaml.h>

// ROS
#include <geometry_msgs/Pose.h>
#include <ros/package.h>  // getPath
#include <shape_msgs/SolidPrimitive.h>

// MOVEIT
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>



//#############################################################################
// NAMESPACE ##################################################################
/// @brief Namespace of data manager.
namespace data_manager {



//#############################################################################
// CONFIGS ####################################################################
/// @brief This namespace contains the configurations of this file.
namespace config {
const std::string FRAME_REF = "panda_link0";
const std::string PACKAGE_NAME = "panda_controller";
const std::string DATA_FOLDER_NAME = "/data/";
const std::string MESHES_FOLDER_NAME = DATA_FOLDER_NAME + "meshes/";
const std::string POSES_RELATIVE = DATA_FOLDER_NAME + "poses.yaml";
const std::string SCENES_RELATIVE = DATA_FOLDER_NAME + "scenes.yaml";
}  // namespace config



//#############################################################################
// FUNCTIONS INTERFACE ########################################################

/**
 * @brief Saves the pose with the specified name in the Data folder (e.g.:
 * data/poses.yaml).
 *
 * @param POSE The pose to be saved.
 * @param EEF The end effector link name reference.
 * @param NAME The name of the pose.
 */
void save_pose(const geometry_msgs::Pose &POSE, const std::string &EEF,
               const std::string &NAME = "current");

/**
 * @brief Get the pose object with the name specified from Data folder (e.g.:
 * data/poses.yaml).
 *
 * @param NAME The name of the pose.
 * @return geometry_msgs::Pose The desired Pose object.
 */
geometry_msgs::Pose get_pose(const std::string &NAME);


/**
 * @brief Get the scene object with the name specified from Data folder (e.g.:
 * data/scenes.yaml).
 *
 * @param NAME The name of scene.
 * @return moveit_msgs::PlanningScene The desired PlanningScene object.
 */
moveit_msgs::PlanningScene get_scene(const std::string &NAME);

}  // namespace data_manager