#pragma once

// MY LIBS
#include "my_exceptions.hpp"

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



//#############################################################################
// CLASSES
namespace arm {

class Panda {
  private:
    moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

  public:
    // Constructors
    explicit Panda(const std::string &panda_group);

    // Get current pose
    geometry_msgs::Pose getCurrentPose();

    // Move to the position pose
    void moveToPosition(const geometry_msgs::Pose &pose, const float &speed,
                        const bool &active_moving);

    // Move gripper of len meters
    void moveGripper(const double len, const float &speed,
                     const bool &active_moving);

    // Alternative of moveGripper
    void setGripper(trajectory_msgs::JointTrajectory &posture, bool open);

    // Execute pick
    void pick(const geometry_msgs::Pose &pose);

    // Execute place
    void place(const geometry_msgs::Pose &pose);
};


}  // namespace arm