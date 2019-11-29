#pragma once

// MY LIBS
#include "my_exceptions.hpp"

// ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



//#############################################################################
// CLASSES
namespace arm {


// Constants
const auto FINGER_SX = "panda_finger_joint1";
const auto FINGER_DX = "panda_finger_joint2";
const auto ARM_GROUP = "panda_arm";
const auto FRAME_REF = "panda_link0";
const float GRIPPER_MAX_WIDTH = 0.04;
const float DELTA_GRASP = 0.001;


// Functions
geometry_msgs::Vector3 get_vector_with(const std::string &type);


// Class to easily manage the Panda arm with moveit
class Panda {
  private:
    moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr;
    moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_ptr;
    float speed = 1.0;

  public:
    // Constructors
    explicit Panda(const std::string &PANDA_GROUP = ARM_GROUP);

    // Get current pose
    geometry_msgs::Pose getCurrentPose();

    // Set speed
    void setSpeed(const float &SPEED);

    // Set scene
    void setScene(const moveit_msgs::PlanningScene &SCENE);

    // Reset scene
    void resetScene();

    // Move to the position pose
    void moveToPosition(const geometry_msgs::Pose &POSE,
                        const bool &PLAN_ONLY = false);

    // Execute pick
    void pick(const std::string &OBJECT_NAME, const std::string &PICK_SURFACE,
              const geometry_msgs::Vector3 &pre_vector,
              const geometry_msgs::Vector3 &post_vector,
              const geometry_msgs::Pose &POSE, const bool &PLAN_ONLY = false);

    // Execute place
    void place(const std::string &OBJECT_NAME, const std::string &PLACE_SURFACE,
               const geometry_msgs::Vector3 &pre_vector,
               const geometry_msgs::Vector3 &post_vector,
               const geometry_msgs::Pose &POSE, const bool &PLAN_ONLY = false);
};

}  // namespace arm