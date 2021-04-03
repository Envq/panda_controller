#pragma once

// ROS and Moveit
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>            //tf2::Quaternion
#include <tf2/convert.h>                          //tf2::conversion
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  //tf2::conversion
#include <tf2_ros/transform_listener.h>

// BOOST
#include <boost/shared_ptr.hpp>

// Custom
#include "panda_controller/data_manager.hpp"
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
    std::shared_ptr<tf2_ros::Buffer> _tf_buffer_ptr;
    std::shared_ptr<tf2_ros::TransformListener> _tf_listener_ptr;
    tf2::Transform _tcp_to_flange;


    /**
     * @brief get the world->Flange from the world->TCP.
     *
     * @param TCP_POSE TCP (tool center point) pose.
     * @return geometry_msgs::Pose Flange pose.
     */
    geometry_msgs::Pose _getFlangeFromTCP(const geometry_msgs::Pose &TCP_POSE);


  public:
    /**
     * @brief Construct a new Panda Arm object
     *
     * @param DELAY the seconds of delay for load the TF2
     */
    explicit PandaArm(const float DELAY = 1.0);

    /**
     * @brief Get the MoveGroupInterfacePtrobject. (Remember that this object is
     * deleted along with this class)
     *
     * @return moveit::planning_interface::MoveGroupInterfacePtr move group
     * "panda_arm"
     */
    moveit::planning_interface::MoveGroupInterfacePtr getMoveGroup();

    /**
     * @brief Set the maximum velocity scaling factor.
     *
     * @param VELOCITY_FACTOR maximum velocity scaling factor.
     */
    void setMaxVelocityScalingFactor(const float VELOCITY_FACTOR);


    /**
     * @brief Get the joints value
     *
     * @return std::vector<double> vector of joints value
     */
    std::vector<double> getJoints();


    /**
     * @brief Move the specified joint in radiants.
     *
     * @param JOINTS target joints value.
     * @param ADJUST_IN_BOUNDS If the specified values exceed the joint
     * limits, the maximum bounds are assumed instead.
     */
    void moveToJoints(const std::vector<double> &JOINTS,
                      const bool ADJUST_IN_BOUNDS = false);


    /**
     * @brief Move the arm in 'Ready' Pose.
     *
     */
    void moveToReady();


    /**
     * @brief Get the current Pose of the TCP (tool center point).
     *
     * @return geometry_msgs::Pose The Pose object of current pose.
     */
    geometry_msgs::Pose getPose();


    /**
     * @brief Move the TCP (tool center point) in this pose.
     *
     * @param POSE The pose where to go.
     */
    void moveToPose(const geometry_msgs::Pose &POSE);


    /**
     * @brief move the TCP (tool center point) relative current pose.
     *
     * @param X offset on X axis.
     * @param Y offset on Y axis.
     * @param Z offset on Z axis.
     */
    void relativeMovePos(const double X, const double Y, const double Z);


    /**
     * @brief move the TCP (tool center point) relative current pose (in
     * degree).
     *
     * @param ROLL offset around X axis.
     * @param PITCH offset around Y axis.
     * @param YAW offset around Z axis.
     */
    void relativeMoveRPY(const double ROLL, const double PITCH,
                         const double YAW);


    /**
     * @brief Move the TCP (tool center point) through the waypoints.
     *
     * @param WAYPOINTS Poses that the Cartesian path must follow.
     * @param EEF_STEP The step size of at most eef_step meters between end
     * effector. configurations of consecutive points in the result
     * trajectory.
     * @param JUMP_THRESHOLD No more than jump_threshold is allowed as
     * change in distance in the configuration space of the robot (this is
     * to prevent 'jumps' in IK solutions).
     */
    void waypointsMove(const std::vector<geometry_msgs::Pose> &WAYPOINTS,
                       const double EEF_STEP = 0.01,
                       const double JUMP_THRESHOLD = 0.0);


    /**
     * @brief Move the TCP (tool center point) linearly towards the pose.
     *
     * @param POSE The target pose.
     * @param EEF_STEP The step size of at most eef_step meters between end
     * effector. configurations of consecutive points in the result
     * trajectory.
     * @param JUMP_THRESHOLD No more than jump_threshold is allowed as
     * change in distance in the configuration space of the robot (this is
     * to prevent 'jumps' in IK solutions).
     */
    void linearMove(const geometry_msgs::Pose &POSE,
                    const double EEF_STEP = 0.01,
                    const double JUMP_THRESHOLD = 0.0);


    /**
     * @brief Save in data folder the current TCP (tool center point) pose.
     *
     * @param POSE_NAME pose name.
     */
    void savePose(const std::string &POSE_NAME = "last");


    /**
     * @brief Move the TCP (tool center point) in the saved POSE_NAME .
     *
     * @param POSE_NAME pose name.
     */
    void moveToPose(const std::string &POSE_NAME);


    /**
     * @brief Get the TCP (tool center point) in the saved POSE_NAME .
     *
     * @param POSE_NAME pose name.
     * @return geometry_msgs::Pose The Pose object in the saved POSE_NAME.
     */
    geometry_msgs::Pose getPose(const std::string &POSE_NAME);
};

}  // namespace panda_controller