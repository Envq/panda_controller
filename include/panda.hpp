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
namespace robot {


//#############################################################################
// CONSTANTS ##################################################################
const std::string FRAME_REF = "panda_link0";
const double GRIPPER_MAX_WIDTH = 0.08;
const float DEFAULT_ARM_SPEED = 0.1;
const float DEFAULT_GRIPPER_SPEED = 0.1;
const double DEFAULT_MOVE_STEP = 0.01;
const double DEFAULT_MOVE_JUMP_THRESHOLD = 0.0;
const double DISTANCE_FROM_GRIPPER_CENTER = 0.105;
const auto HOME_JOINTS = std::vector<double>{
    0.00, -0.25 * M_PI, 0.00, -0.75 * M_PI, 0.00, 0.50 * M_PI, 0.25 * M_PI};



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

    // Move to the specified pose
    void moveToPose(const geometry_msgs::Pose &POSE,
                    const bool &PLAN_ONLY = false);

    // Move joints
    void moveJoints(const std::vector<double> &JOINTS,
                    const bool &PLAN_ONLY = false);

    // Move the specified joint in radiants
    void moveJointRad(const int &JOINT, const double &VAL,
                      const bool &PLAN_ONLY = false);

    // Move the specified joint in degrees
    void moveJointDeg(const int &JOINT, const double &VAL,
                      const bool &PLAN_ONLY = false);

    // Go to ready pose
    void moveToReadyPose();

    // Move in cartesian path
    void cartesianMovement(
        const std::vector<geometry_msgs::Pose> &WAYPOINTS,
        const double &STEP = DEFAULT_MOVE_STEP,
        const double &JUMP_THRESHOLD = DEFAULT_MOVE_JUMP_THRESHOLD,
        const bool &PLAN_ONLY = false);

    // Move in cartesian path in only one waypoint
    void cartesianMovement(
        const geometry_msgs::Pose &POSE, const double &STEP = DEFAULT_MOVE_STEP,
        const double &JUMP_THRESHOLD = DEFAULT_MOVE_JUMP_THRESHOLD,
        const bool &PLAN_ONLY = false);

    // Perfom pick without constraints
    void pick(const geometry_msgs::Pose &POSE, const double &GRASP_WIDTH,
              const double &GRASP_FORCE, const double &GRASP_EPSILON_INNER,
              const double &GRASP_EPSILON_OUTER, const bool &PLAN_ONLY = false);

    // Perform pick with pre and post approch
    void pick(const geometry_msgs::Pose &POSE,
              const geometry_msgs::Vector3 &PRE_GRASP_APPROCH,
              const double &GRASP_WIDTH, const double &GRASP_FORCE,
              const double &GRASP_EPSILON_INNER,
              const double &GRASP_EPSILON_OUTER, const bool &PLAN_ONLY = false);

    // Perform pick with collision object
    void pick(const geometry_msgs::Pose &POSE, const std::string &OBJECT_NAME,
              const double &GRASP_FORCE, const double &GRASP_EPSILON_INNER,
              const double &GRASP_EPSILON_OUTER, const bool &PLAN_ONLY = false);

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
    void gripperMove(const double &WIDTH);

    // Grasp gripper
    void gripperGrasp(const double &WIDTH, const double &GRASP_FORCE,
                      const double &GRASP_EPSILON_INNER,
                      const double &GRASP_EPSILON_OUTER);
};


}  // namespace robot