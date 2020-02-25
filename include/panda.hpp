/**
 * @file panda.hpp
 * @author Enrico Sgarbanti
 * @brief This file contains the Panda Franka Emika robot management class.
 * @version 0.1
 * @date 20-02-2020
 *
 * @copyright Copyright (c) 2020 by Enrico Sgarbanti. License GPLv3.
 *
 */
#pragma once

// PANDA CONTROLLER
#include "exceptions.hpp"  //PCEXC

// ROS
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MOVEIT
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// FRANKA_GRIPPER
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/GraspGoal.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/HomingGoal.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/MoveGoal.h>

// BOOST
#include <boost/shared_ptr.hpp>



//#############################################################################
// NAMESPACE ##################################################################
/// @brief Namespace of Panda robot.
namespace robot {



//#############################################################################
// CONFIGS ####################################################################
/// @brief This namespace contains the configurations of this file.
namespace config {
const std::string FRAME_REF = "panda_link0";
const double GRIPPER_MAX_WIDTH = 0.08;
const auto READY_JOINTS = std::vector<double>{
    0.00, -0.25 * M_PI, 0.00, -0.75 * M_PI, 0.00, 0.50 * M_PI, 0.25 * M_PI};
}  // namespace config



//#############################################################################
// DEFAULT VALUES #############################################################
/// @brief This namespace contains the default values.
namespace defaults {
const float ARM_SPEED = 1.0;
const float GRIPPER_SPEED = 1.0;
const float STEP = 0.01;
const float JUMP_THRESHOLD = 0.0;
}  // namespace defaults



//#############################################################################
// TYPEDEF ####################################################################
/// @brief ROS action client for gripper homing.
typedef actionlib::SimpleActionClient<franka_gripper::HomingAction>
    GripperHomingClient;
/// @brief ROS action client for gripper move.
typedef actionlib::SimpleActionClient<franka_gripper::MoveAction>
    GripperMoveClient;
/// @brief ROS action client for gripper grasp.
typedef actionlib::SimpleActionClient<franka_gripper::GraspAction>
    GripperGraspClient;



//#############################################################################
// CLASSES ####################################################################
/// @brief The Panda Franka Emika robot management class.
class Panda {
  private:
    moveit::planning_interface::MoveGroupInterfacePtr arm_ptr_;
    moveit::planning_interface::MoveGroupInterfacePtr hand_ptr_;
    moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_ptr_;
    boost::shared_ptr<GripperHomingClient> gripper_homing_client_ptr_;
    boost::shared_ptr<GripperMoveClient> gripper_move_client_ptr_;
    boost::shared_ptr<GripperGraspClient> gripper_grasp_client_ptr_;
    float gripper_speed_;
    bool gripper_is_active_;

  public:
    /**
     * @brief Construct a new Panda object.
     *
     * @param GRIPPER_IS_ACTIVE Used for disable the gripper actions (default is
     * true).
     */
    explicit Panda(const bool &GRIPPER_IS_ACTIVE = true);

    /**
     * @brief Get the current Pose.
     *
     * @return geometry_msgs::Pose The Pose object of current pose.
     */
    geometry_msgs::Pose getCurrentPose();

    /**
     * @brief Set the arm speed.
     *
     * @param SPEED The speed value.
     */
    void setArmSpeed(const float &SPEED);

    /**
     * @brief Set the Gripper Speed object.
     *
     * @param SPEED The speed value.
     */
    void setGripperSpeed(const float &SPEED);

    /**
     * @brief Load the scene.
     *
     * @param SCENE The scene to use.
     */
    void setScene(const moveit_msgs::PlanningScene &SCENE);

    /**
     * @brief Load a empty scene.
     *
     */
    void resetScene();

    /**
     * @brief Move the joints in the angles specified (radiants).
     *
     * @param JOINTS the vector of angles.
     * @param ADJUST_IN_BOUNDS If the specified values exceed the joint limits,
     * the maximum bounds are assumed instead.
     */
    void moveJointsTo(const std::vector<double> &JOINTS,
                      const bool &ADJUST_IN_BOUNDS = true);

    /**
     * @brief Move the specified joint in radiants.
     *
     * @param JOINT The name of the joint to be moved (1 to 7).
     * @param VAL The value of angle.
     * @param ADJUST_IN_BOUNDS If the specified values exceed the joint limits,
     * the maximum bounds are assumed instead.
     */
    void moveJointRad(const int &JOINT, const double &VAL,
                      const bool &ADJUST_IN_BOUNDS = true);

    /**
     * @brief Move the specified joint in degrees.
     *
     * @param JOINT The name of the joint to be moved (1 to 7).
     * @param VAL The value of angle.
     * @param ADJUST_IN_BOUNDS If the specified values exceed the joint limits,
     * the maximum bounds are assumed instead.
     */
    void moveJointDeg(const int &JOINT, const double &VAL,
                      const bool &ADJUST_IN_BOUNDS = true);

    /**
     * @brief Move the arm in 'Ready' Pose.
     *
     */
    void moveToReadyPose();

    /**
     * @brief Move the arm in the specified pose.
     *
     * @param POSE The pose where to go.
     */
    void moveToPose(const geometry_msgs::Pose &POSE);

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
    void
    cartesianMovement(const std::vector<geometry_msgs::Pose> &WAYPOINTS,
                      const double &STEP = defaults::STEP,
                      const double &JUMP_THRESHOLD = defaults::JUMP_THRESHOLD);

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
    void
    cartesianMovement(const geometry_msgs::Pose &POSE,
                      const double &STEP = defaults::STEP,
                      const double &JUMP_THRESHOLD = defaults::JUMP_THRESHOLD);

    /**
     * @brief Pick up an object with pre-grasp-approch. Moves the arm to the
     * desired position, starting from the pre-approach-pose, and proceeds
     * moving linearly with respect to the target.
     *
     * @param POSE The pose where the robot grasp object.
     * @param PRE_GRASP_APPROCH The vector of grasp-pre-approch. It contains the
     * offset from target pose in meters.
     * @param GRASP_WIDTH The width value for gripperGrasp().
     * @param GRASP_FORCE The force value for gripperGrasp().
     * @param GRASP_EPSILON_INNER The epsilon_inner value for gripperGrasp().
     * @param GRASP_EPSILON_OUTER The epsilon_outer value for gripperGrasp().
     * @param STEP Step value for cartesianMovement().
     * @param JUMP_THRESHOLD Jumpo threshold value for cartesianMovement().
     */
    void pick(const geometry_msgs::Pose &POSE,
              const geometry_msgs::Vector3 &PRE_GRASP_APPROCH,
              const double &GRASP_WIDTH, const double &GRASP_FORCE,
              const double &GRASP_EPSILON_INNER,
              const double &GRASP_EPSILON_OUTER,
              const double &STEP = defaults::STEP,
              const double &JUMP_THRESHOLD = defaults::JUMP_THRESHOLD);

    /**
     * @brief Pick up the object specified.
     *
     * @param POSE The pose where the robot grasp object.
     * @param OBJECT_NAME The name of the target object.
     * @param GRASP_WIDTH The width value for gripperGrasp().
     * @param GRASP_FORCE The force value for gripperGrasp().
     * @param GRASP_EPSILON_INNER The epsilon_inner value for gripperGrasp().
     * @param GRASP_EPSILON_OUTER The epsilon_outer value for gripperGrasp().
     */
    void pick(const geometry_msgs::Pose &POSE, const std::string &OBJECT_NAME,
              const double &GRASP_WIDTH, const double &GRASP_FORCE,
              const double &GRASP_EPSILON_INNER,
              const double &GRASP_EPSILON_OUTER);

    /**
     * @brief Place the object with post-grasp-retreat and post-place-retreat.
     * Moves the arm linearly to post-grasp-pose and then move to the desired
     * position. After that proceeds moving linearly in the post-place-pose.
     *
     * @param POSE The object to the pose specified.
     * @param POST_GRASP_RETREAT The vector of post-grasp-retreat. It contains
     * the offset from current pose in meters.
     * @param POST_PLACE_RETREAT The vector of post-place-retreat. It contains
     * the offset from target pose in meters.
     * @param STEP Step value for cartesianMovement().
     * @param JUMP_THRESHOLD Jumpo threshold value for cartesianMovement().
     */
    void place(const geometry_msgs::Pose &POSE,
               const geometry_msgs::Vector3 &POST_GRASP_RETREAT,
               const geometry_msgs::Vector3 &POST_PLACE_RETREAT,
               const double &STEP = defaults::STEP,
               const double &JUMP_THRESHOLD = defaults::JUMP_THRESHOLD);

    /**
     * @brief Place the object to the pose specified.
     *
     * @param POSE The pose in which place the object
     */
    void place(const geometry_msgs::Pose &POSE);

    /**
     * @brief Homes the gripper and updates the maximum width given the mounted
     * fingers.
     *
     */
    void gripperHoming();

    /**
     * @brief Moves to a target WIDTH.
     *
     * @param WIDTH Target width.
     */
    void gripperMove(const double &WIDTH);

    /**
     * @brief Tries to grasp at the desired width with a desired force while
     * closing. The operation is successful if the distance WIDTH between the
     * gripper fingers is: width−epsilon_inner<width<width+epsilon_outer.
     *
     * @param WIDTH Distante between the gripper fingers.
     * @param FORCE Desired force applied while closing fingers.
     * @param EPSILON_INNER The value of epsilon inner.
     * @param EPSILON_OUTER The value of epsilon outer.
     */
    void gripperGrasp(const double &WIDTH, const double &FORCE,
                      const double &EPSILON_INNER, const double &EPSILON_OUTER);
};

}  // namespace robot