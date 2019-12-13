#pragma once

// MY LIBS
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
// CLASSES
namespace robot {


// Constants
const auto FRAME_REF = "panda_link0";
const float GRIPPER_MAX_WIDTH = 0.08;


// Typedef
typedef actionlib::SimpleActionClient<franka_gripper::HomingAction>
    GripperHomingClient;
typedef actionlib::SimpleActionClient<franka_gripper::MoveAction>
    GripperMoveClient;
typedef actionlib::SimpleActionClient<franka_gripper::GraspAction>
    GripperGraspClient;


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
    explicit Panda();

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
    void pick(const geometry_msgs::Pose &POSE, const std::string &OBJECT_NAME,
              const bool &PLAN_ONLY = false);

    // Execute place
    void place(const geometry_msgs::Pose &POSE, const std::string &OBJECT_NAME,
               const bool &PLAN_ONLY = false);

    // Homing gripper
    void gripperHoming();

    // Move gripper
    void gripperMove(const float &WIDTH, const float &SPEED = 1.0);

    // Grasp gripper
    void gripperGrasp(const float &WIDTH, const float &SPEED = 0.5,
                      const float &FORCE = 20.0,
                      const float &EPSILON_INNER = 0.002,
                      const float &EPSILON_OUTER = 0.002);
};

}  // namespace robot