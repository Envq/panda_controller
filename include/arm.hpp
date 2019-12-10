#pragma once

// MY LIBS
#include "my_exceptions.hpp"

// ROS
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


//#############################################################################
// CLASSES
namespace arm {


// Constants
const auto FRAME_REF = "panda_link0";
const float GRIPPER_MAX_WIDTH = 0.08;
const float EPSILON_GRASP = 0.002;


// Class to easily manage the Panda arm with moveit
class Panda {
  private:
    moveit::planning_interface::MoveGroupInterfacePtr arm_group_ptr;
    moveit::planning_interface::MoveGroupInterfacePtr hand_group_ptr;
    moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_ptr;
    float speed;

  public:
    // Constructors
    explicit Panda();

    // Get current pose
    geometry_msgs::Pose getCurrentPose();

    // Set speed
    void setSpeed(const float &SPEED);

    // Set scene
    void setScene(const moveit_msgs::PlanningScene &SCENE);

    // Set scene
    void resetScene();

    // Move to the position pose
    void moveToPosition(const geometry_msgs::Pose &POSE,
                        const bool &PLAN_ONLY = false);

    // Execute pick
    void pick(const geometry_msgs::Pose &POSE, const std::string &OBJECT_NAME,
              const bool &PLAN_ONLY = false);

    // Execute place
    void place(const geometry_msgs::Pose &POSE, const std::string &OBJECT_NAME,
               const bool &PLAN_ONLY = false);

    // Move gripper
    void moveGripper(const float &WIDTH, const bool &PLAN_ONLY = false);
};

}  // namespace arm