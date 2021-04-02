#pragma once

// C++
#include <fstream>
#include <yaml-cpp/yaml.h>

// ROS
#include <geometric_shapes/shape_operations.h>  //Mesh
#include <geometry_msgs/Pose.h>
#include <ros/package.h>  // getPath
#include <shape_msgs/SolidPrimitive.h>

// MOVEIT
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

// Custom
#include "panda_controller/exceptions.hpp"



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// STRUCTS ====================================================================
struct scene_object {
    moveit_msgs::CollisionObject object;
    moveit_msgs::ObjectColor color;
};



// CONFIGS ====================================================================
/// @brief This namespace contains the configurations of this file.
namespace data_manager {
const std::string PACKAGE_NAME = "panda_controller";
const std::string DATA_FOLDER_NAME = "/data/";
const std::string MESHES_FOLDER_REL_PATH = DATA_FOLDER_NAME + "meshes/";
const std::string POSES_REL_PATH = DATA_FOLDER_NAME + "poses.yaml";
const std::string SCENES_REL_PATH = DATA_FOLDER_NAME + "scenes.yaml";
}  // namespace data_manager



// FUNCTIONS ==================================================================
/**
 * @brief Saves the pose with the specified name in the Data folder (e.g.:
 * data/poses.yaml).
 *
 * @param POSE The pose to be saved.
 * @param NAME The name of the pose.
 */
void save_pose(const geometry_msgs::Pose &POSE, const std::string &NAME);

/**
 * @brief Get the pose object with the name specified from Data folder (e.g.:
 * data/poses.yaml).
 *
 * @param NAME The name of the pose.
 * @return geometry_msgs::Pose The desired Pose object.
 */
geometry_msgs::Pose load_pose(const std::string &NAME);


/**
 * @brief Get the scene object with the name specified from Data folder (e.g.:
 * data/scenes.yaml).
 *
 * @param NAME The name of scene.
 * @return moveit_msgs::PlanningScene The desired PlanningScene object.
 */
moveit_msgs::PlanningScene load_scene(const std::string &NAME);

}  // namespace panda_controller