#pragma once

// ROS and Moveit
#include <actionlib/client/simple_action_client.h>  //franka_gripper
#include <moveit/move_group_interface/move_group_interface.h>

// BOOST
#include <boost/shared_ptr.hpp>

// franka_gripper
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/GraspGoal.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/HomingGoal.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/MoveGoal.h>

// Custom
#include "panda_controller/exceptions.hpp"



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CONFIGS ====================================================================
/// @brief This namespace contains the configurations of this file.
namespace panda_gripper {
const double GRIPPER_MAX_WIDTH = 0.08;
}  // namespace panda_gripper



// TYPEDEF ====================================================================
/// @brief ROS action client for gripper homing.
typedef actionlib::SimpleActionClient<franka_gripper::HomingAction>
    GripperHomingClient;
/// @brief ROS action client for gripper move.
typedef actionlib::SimpleActionClient<franka_gripper::MoveAction>
    GripperMoveClient;
/// @brief ROS action client for gripper grasp.
typedef actionlib::SimpleActionClient<franka_gripper::GraspAction>
    GripperGraspClient;



// CLASSES ====================================================================
/// @brief The Panda robot gripper management class.
class PandaGripper {
  private:
    moveit::planning_interface::MoveGroupInterfacePtr _hand_ptr;
    boost::shared_ptr<GripperHomingClient> _homing_client_ptr;
    boost::shared_ptr<GripperMoveClient> _move_client_ptr;
    boost::shared_ptr<GripperGraspClient> _grasp_client_ptr;
    double _curren_width;
    bool _real_robot;

    /**
     * @brief Moves to a target WIDTH using moveit.
     *
     * @param WIDTH Distante between the gripper fingers.
     */
    void _moveHand(const double &WIDTH);


  public:
    /**
     * @brief Construct a new PandaGripper object.
     *
     * @param REAL_ROBOT If true, it uses franka_gripper.
     */
    explicit PandaGripper(const bool &REAL_ROBOT = false);


    /**
     * @brief Homes the gripper and updates the maximum width given the mounted
     * fingers.
     */
    void homing();


    /**
     * @brief Moves to a target WIDTH.
     *
     * @param WIDTH Distante between the gripper fingers.
     * @param SPEED Desired speed applied while moving fingers.
     */
    void move(const double &WIDTH, const double &SPEED = 0.5);


    /**
     * @brief Tries to grasp at the desired width with a desired force while
     * closing. The operation is successful if the distance WIDTH between the
     * gripper fingers is: width−epsilon_inner < width < width+epsilon_outer.
     *
     * @param WIDTH Distante between the gripper fingers.
     * @param SPEED Desired speed applied while moving fingers.
     * @param FORCE Desired force applied while closing fingers.
     * @param EPSILON_INNER The value of epsilon inner.
     * @param EPSILON_OUTER The value of epsilon outer.
     */
    void grasp(const double &WIDTH, const double &SPEED = 0.5,
               const double &FORCE = 20, const double &EPSILON_INNER = 0.02,
               const double &EPSILON_OUTER = 0.02);
};

}  // namespace panda_controller