#pragma once

// ROS and Moveit
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Custom
#include "panda_controller/exceptions.hpp"



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CONFIGS ====================================================================
/// @brief This namespace contains the configurations of this file.
namespace panda_arm {
const auto READY_JOINTS = std::vector<double>{
    0.00, -0.25 * M_PI, 0.00, -0.75 * M_PI, 0.00, 0.50 * M_PI, 0.25 * M_PI};
}  // namespace panda_arm



// CLASSES ====================================================================
/// @brief The Panda robot arm management class.
class PandaArm {
  private:
    moveit::planning_interface::MoveGroupInterfacePtr _arm_ptr;

  public:
    /**
     * @brief Construct a new PandaArm object.
     *
     * @param VELOCITY_FACTOR maximum velocity scaling factor.
     */
    explicit PandaArm(const double VELOCITY_FACTOR = 0.1);


    /**
     * @brief Set the maximum velocity scaling factor.
     *
     * @param VELOCITY_FACTOR maximum velocity scaling factor.
     */
    void setMaxVelocityScalingFactor(const float &VELOCITY_FACTOR);


    /**
     * @brief Get the joints value
     *
     * @return std::vector<double> vector of joints value
     */
    std::vector<double> getArmJoints();


    /**
     * @brief Move the specified joint in radiants.
     *
     * @param JOINTS target joints value.
     * @param ADJUST_IN_BOUNDS If the specified values exceed the joint limits,
     * the maximum bounds are assumed instead.
     */
    void moveArmJoints(const std::vector<double> &JOINTS,
                       const bool &ADJUST_IN_BOUNDS = false);


    /**
     * @brief Move the arm in 'Ready' Pose.
     *
     */
    void moveArmReady();


    /**
     * @brief Get the current Pose.
     *
     * @return geometry_msgs::Pose The Pose object of current pose.
     */
    geometry_msgs::Pose getArmPose();


    /**
     * @brief Move the arm in the specified pose.
     *
     * @param POSE The pose where to go.
     */
    void moveArmPose(const geometry_msgs::Pose &POSE);


    /**
     * @brief Move the arm in cartesian path with waypoints.
     *
     * @param WAYPOINTS Poses that the Cartesian path must follow.
     * @param STEP The step size of at most eef_step meters between end
     * effector. configurations of consecutive points in the result trajectory.
     * @param JUMP_THRESHOLD No more than jump_threshold is allowed as change in
     * distance in the configuration space of the robot (this is to prevent
     * 'jumps' in IK solutions).
     */
    void cartesianMovement(const std::vector<geometry_msgs::Pose> &WAYPOINTS,
                           const double &STEP = 0.01,
                           const double &JUMP_THRESHOLD = 0.0);


    /**
     * @brief Move the arm in cartesian path with target point.
     *
     * @param POSE The target pose.
     * @param STEP The step size of at most eef_step meters between end
     * effector. configurations of consecutive points in the result trajectory.
     * @param JUMP_THRESHOLD No more than jump_threshold is allowed as change in
     * distance in the configuration space of the robot (this is to prevent
     * 'jumps' in IK solutions).
     */
    void cartesianMovement(const geometry_msgs::Pose &POSE,
                           const double &STEP = 0.01,
                           const double &JUMP_THRESHOLD = 0.0);
};
}  // namespace panda_controller