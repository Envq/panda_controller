#pragma once

// PANDA CONTROLLER
#include "exceptions.hpp"  //PCEXC

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
// CONSTANTS ##################################################################
const std::string FRAME_REF = "panda_link0";
const float GRIPPER_MAX_WIDTH = 0.08;
const float DEFAULT_ARM_SPEED = 1.0;
const float DEFAULT_GRIPPER_SPEED = 0.1;
const float DEFAULT_MOVE_STEP = 0.01;
const float DEFAULT_MOVE_JUMP_THRESHOLD = 0.0;



//#############################################################################
// TYPEDEF ####################################################################
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
    moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr_;
    moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_ptr_;
    boost::shared_ptr<GripperHomingClient> gripper_homing_client_ptr_;
    boost::shared_ptr<GripperMoveClient> gripper_move_client_ptr_;
    boost::shared_ptr<GripperGraspClient> gripper_grasp_client_ptr_;
    float gripper_speed_;

  public:
    // Constructors
    explicit Panda(const bool &HOMING_STARTUP = false);

    // Get current pose
    geometry_msgs::Pose getCurrentPose();

    // Set arm speed
    void setArmSpeed(const float &SPEED);

    // Set gripper speed
    void setGripperSpeed(const float &SPEED);

    // Set scene
    void setScene(const moveit_msgs::PlanningScene &SCENE);

    // Set scene
    void resetScene();

    // Move to the position pose
    void moveToPose(const geometry_msgs::Pose &POSE,
                        const bool &PLAN_ONLY = false);

    // Move in cartesian path
    void cartesianMovement(const std::vector<geometry_msgs::Pose> &WAYPOINTS,
                           const double &STEP = 0.01,
                           const double &JUMP_THRESHOLD = 0.0,
                           const bool &PLAN_ONLY = false);

    // Move in cartesian path in only one waypoint
    void cartesianMovement(
        const geometry_msgs::Pose &POSE, const double &STEP = DEFAULT_MOVE_STEP,
        const double &JUMP_THRESHOLD = DEFAULT_MOVE_JUMP_THRESHOLD,
        const bool &PLAN_ONLY = false);

    // Perfom pick without constraints
    void pick(const geometry_msgs::Pose &POSE, const float &GRASP_WIDTH,
              const float &GRASP_FORCE, const float &GRASP_EPSILON_INNER,
              const float &GRASP_EPSILON_OUTER, const bool &PLAN_ONLY = false);

    // Perform pick with pre and post approch
    void pick(const geometry_msgs::Pose &POSE,
              const geometry_msgs::Vector3 &PRE_GRASP_APPROCH,
              const float &GRASP_WIDTH, const float &GRASP_FORCE,
              const float &GRASP_EPSILON_INNER,
              const float &GRASP_EPSILON_OUTER, const bool &PLAN_ONLY = false);

    // Perform pick with collision object
    void pick(const geometry_msgs::Pose &POSE, const std::string &OBJECT_NAME,
              const float &GRASP_FORCE, const float &GRASP_EPSILON_INNER,
              const float &GRASP_EPSILON_OUTER, const bool &PLAN_ONLY = false);

    // Perform place without constraints
    void place(const geometry_msgs::Pose &POSE, const bool &PLAN_ONLY = false);

    // Perform place with pre and post approch
    void place(const geometry_msgs::Pose &POSE,
               const geometry_msgs::Vector3 &POST_GRASP_RETREAT,
               const geometry_msgs::Vector3 &POST_PLACE_RETREAT,
               const bool &PLAN_ONLY = false);

    // Homing gripper
    void gripperHoming();

    // Move gripper
    void gripperMove(const float &WIDTH);

    // Grasp gripper
    void gripperGrasp(const float &WIDTH, const float &GRASP_FORCE,
                      const float &GRASP_EPSILON_INNER,
                      const float &GRASP_EPSILON_OUTER);
};


}  // namespace robot