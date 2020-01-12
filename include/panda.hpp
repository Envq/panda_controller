#pragma once

// PANDA CONTROLLER
#include "my_exceptions.hpp"

// ROS
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

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
namespace robot {


//#############################################################################
// CONSTANTS AND TYPEDEF ######################################################
const auto FRAME_REF = "panda_link0";
const float GRIPPER_MAX_WIDTH = 0.08;
const float DEFAULT_ARM_SPEED = 1.0;
const float DEFAULT_GRIPPER_SPEED = 0.1;
const float DEFAULT_GRIPPER_FORCE = 10.0;

typedef actionlib::SimpleActionClient<franka_gripper::HomingAction>
    GripperHomingClient;
typedef actionlib::SimpleActionClient<franka_gripper::MoveAction>
    GripperMoveClient;
typedef actionlib::SimpleActionClient<franka_gripper::GraspAction>
    GripperGraspClient;



//#############################################################################
// CLASSES ####################################################################
// Class to easily manage the Panda arm with moveit
class Panda {
  private:
    moveit::planning_interface::MoveGroupInterfacePtr arm_group_ptr;
    moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_ptr;
    boost::shared_ptr<GripperHomingClient> gripper_homing_client_ptr;
    boost::shared_ptr<GripperMoveClient> gripper_move_client_ptr;
    boost::shared_ptr<GripperGraspClient> gripper_grasp_client_ptr;

  public:
    // Constructors
    explicit Panda(const bool &HOMING_STARTUP = false);

    // Get current pose
    geometry_msgs::Pose getCurrentPose();

    // Set speed
    void setArmSpeed(const float &SPEED);

    // Set scene
    void setScene(const moveit_msgs::PlanningScene &SCENE);

    // Set scene
    void resetScene();

    // Move to the position pose
    void moveToPosition(const geometry_msgs::Pose &POSE,
                        const bool &PLAN_ONLY = false);

    // Execute pick
    void pick(const geometry_msgs::Pose &POSE, const float &WIDTH,
              const bool &PLAN_ONLY = false);

    // Execute place
    void place(const geometry_msgs::Pose &POSE, const bool &PLAN_ONLY = false);

    // Homing gripper
    void gripperHoming();

    // Move gripper
    void gripperMove(const float &WIDTH,
                     const float &SPEED = robot::DEFAULT_GRIPPER_SPEED);

    // Grasp gripper
    void gripperGrasp(const float &WIDTH,
                      const float &SPEED = robot::DEFAULT_GRIPPER_SPEED,
                      const float &FORCE = robot::DEFAULT_GRIPPER_FORCE,
                      const float &EPSILON_INNER = 0.002,
                      const float &EPSILON_OUTER = 0.002);
};


}  // namespace robot