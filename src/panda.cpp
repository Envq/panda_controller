// PANDA CONTROLLER
#include "panda.hpp"



//#############################################################################
// PRIVATE FUNCTIONS ##########################################################
float get_size_object(
    moveit::planning_interface::PlanningSceneInterfacePtr &planning_scene_ptr,
    const std::string &OBJECT_NAME);



//#############################################################################
// PUBLIC METHODS IMPLEMENTATIONS #############################################
namespace robot {

Panda::Panda(const bool &HOMING_STARTUP) {
    // Init MoveGroupInterface with arm
    try {
        arm_group_ptr.reset(
            new moveit::planning_interface::MoveGroupInterface("panda_arm"));

    } catch (const std::runtime_error &e) {
        throw PCEXC::panda_error(
            "Panda::Panda()" + PCEXC::DIVISOR +
            "Impossible initialize MoveGroupInterface with "
            "'panda_arm'");
    }

    // Init PlanningSceneInterface
    try {
        planning_scene_ptr.reset(
            new moveit::planning_interface::PlanningSceneInterface);

    } catch (const std::runtime_error &e) {
        throw PCEXC::panda_error(
            "Panda::Panda()" + PCEXC::DIVISOR +
            "Impossible initialize PlanningSceneInterface");
    }

    // Init Gripper
    try {
        // Create action client for homing
        gripper_homing_client_ptr.reset(
            new GripperHomingClient("franka_gripper/homing", true));

        // Create action client for move
        gripper_move_client_ptr.reset(
            new GripperMoveClient("franka_gripper/move", true));

        // Create action client for grasp
        gripper_grasp_client_ptr.reset(
            new GripperGraspClient("franka_gripper/grasp", true));

    } catch (const std::runtime_error &e) {
        throw PCEXC::panda_error("Panda::Panda()" + PCEXC::DIVISOR +
                                 "Impossible create "
                                 "action clients for franka_gripper");
    }

    // Set default arm speed
    setArmSpeed(robot::DEFAULT_ARM_SPEED);

    // Homing gripper
    if (HOMING_STARTUP)
        gripperHoming();
}


geometry_msgs::Pose Panda::getCurrentPose() {
    return arm_group_ptr->getCurrentPose().pose;
}


void Panda::setArmSpeed(const float &SPEED) {
    arm_group_ptr->setMaxVelocityScalingFactor(SPEED);
}


void Panda::setScene(const moveit_msgs::PlanningScene &SCENE) {
    planning_scene_ptr->applyPlanningScene(SCENE);
}


void Panda::resetScene() {
    moveit_msgs::PlanningScene scene_empty;
    planning_scene_ptr->applyPlanningScene(scene_empty);
}


void Panda::moveToPosition(const geometry_msgs::Pose &POSE,
                           const bool &PLAN_ONLY) {
    // Set the target Pose
    arm_group_ptr->setPoseTarget(POSE);

    // Perform the planning
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto res = arm_group_ptr->plan(plan);

    // State of planning check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PCEXC::panda_arm_error("Panda::moveToPosition()" +
                                     PCEXC::DIVISOR + "Planning failure");

    // Execute the move
    if (!PLAN_ONLY)
        arm_group_ptr->move();
}


void Panda::pick(const geometry_msgs::Pose &POSE, const float &WIDTH,
                 const bool &PLAN_ONLY) {
    try {
        // Open gripper
        gripperMove(robot::GRIPPER_MAX_WIDTH);

        // Move arm
        moveToPosition(POSE);

        // Close gripper
        gripperGrasp(WIDTH);

        // Post grasp
        auto target_pose = getCurrentPose();
        target_pose.position.z += 0.10;
        moveToPosition(target_pose);

    } catch (PCEXC::panda_arm_error &e) {
        throw PCEXC::panda_arm_error("Panda::pick()" + PCEXC::DIVISOR + "" +
                                     std::string(e.what()));

    } catch (PCEXC::panda_gripper_error &e) {
        throw PCEXC::panda_gripper_error("Panda::place() >>" +
                                         std::string(e.what()));
    }
}


void Panda::place(const geometry_msgs::Pose &POSE, const bool &PLAN_ONLY) {
    try {
        // Move arm
        moveToPosition(POSE);

        // Open gripper
        gripperMove(robot::GRIPPER_MAX_WIDTH);

        // Detach object
        // arm_group_ptr->detachObject(OBJECT_NAME);

    } catch (PCEXC::panda_arm_error &e) {
        throw PCEXC::panda_arm_error("Panda::place() >>" +
                                     std::string(e.what()));

    } catch (PCEXC::panda_gripper_error &e) {
        throw PCEXC::panda_gripper_error("Panda::place() >>" +
                                         std::string(e.what()));
    }
}


void Panda::gripperHoming() {
    // Check if server is connected
    if (!gripper_homing_client_ptr->isServerConnected()) {
        throw PCEXC::panda_gripper_error(
            "Panda::gripperHoming()" + PCEXC::DIVISOR + "isServerConnected()" +
            PCEXC::DIVISOR + "Server is not connected");
    }

    // Wait for action server
    if (!gripper_homing_client_ptr->waitForServer()) {
        throw PCEXC::panda_gripper_error("Panda::gripperHoming()" +
                                         PCEXC::DIVISOR + "waitForServer()" +
                                         PCEXC::DIVISOR + "Timeout");
    }

    // Create homing goal
    auto goal = franka_gripper::HomingGoal();
    gripper_homing_client_ptr->sendGoal(goal);

    // Wait for result
    if (!gripper_homing_client_ptr->waitForResult()) {
        throw PCEXC::panda_gripper_error("Panda::gripperHoming()" +
                                         PCEXC::DIVISOR + "waitForResult()" +
                                         PCEXC::DIVISOR + "Timeout");
    }
}


void Panda::gripperMove(const float &WIDTH, const float &SPEED) {
    // Check if server is connected
    if (!gripper_homing_client_ptr->isServerConnected()) {
        throw PCEXC::panda_gripper_error(
            "Panda::gripperHoming()" + PCEXC::DIVISOR + "isServerConnected()" +
            PCEXC::DIVISOR + "Server is not connected");
    }

    // Wait for action server
    if (!gripper_move_client_ptr->waitForServer()) {
        throw PCEXC::panda_gripper_error("Panda::gripperMove()" +
                                         PCEXC::DIVISOR + "waitForServer()" +
                                         PCEXC::DIVISOR + "Timeout");
    }

    // Create move goal
    auto goal = franka_gripper::MoveGoal();
    goal.width = WIDTH;
    goal.speed = SPEED;
    gripper_move_client_ptr->sendGoal(goal);

    // Wait for result
    if (!gripper_move_client_ptr->waitForResult()) {
        throw PCEXC::panda_gripper_error("Panda::gripperMove()" +
                                         PCEXC::DIVISOR + "waitForResult()" +
                                         PCEXC::DIVISOR + "Timeout");
    }
}


void Panda::gripperGrasp(const float &WIDTH, const float &SPEED,
                         const float &FORCE, const float &EPSILON_INNER,
                         const float &EPSILON_OUTER) {
    // Check if server is connected
    if (!gripper_homing_client_ptr->isServerConnected()) {
        throw PCEXC::panda_gripper_error(
            "Panda::gripperHoming()" + PCEXC::DIVISOR + "isServerConnected()" +
            PCEXC::DIVISOR + "Server is not connected");
    }

    // Wait for action server
    if (!gripper_grasp_client_ptr->waitForServer()) {
        throw PCEXC::panda_gripper_error("Panda::gripperGrasp()" +
                                         PCEXC::DIVISOR + "waitForServer()" +
                                         PCEXC::DIVISOR + "Timeout");
    }

    // Create move goal
    auto goal = franka_gripper::GraspGoal();
    goal.width = WIDTH;
    goal.speed = SPEED;
    goal.force = FORCE;
    goal.epsilon.inner = EPSILON_INNER;
    goal.epsilon.outer = EPSILON_OUTER;
    gripper_grasp_client_ptr->sendGoal(goal);

    // Wait for result
    if (!gripper_grasp_client_ptr->waitForResult()) {
        throw PCEXC::panda_gripper_error("Panda::gripperGrasp()" +
                                         PCEXC::DIVISOR + "waitForResult()" +
                                         PCEXC::DIVISOR + "Timeout");
    }
}

}  // namespace robot



//#############################################################################
// PRIVATE IMPREMENTATIONS
float get_size_object(
    moveit::planning_interface::PlanningSceneInterfacePtr &planning_scene_ptr,
    const std::string &OBJECT_NAME) {
    // Get the attach objects
    auto attach_objects = planning_scene_ptr->getAttachedObjects();

    // Get dimensions object
    auto &dim = attach_objects[OBJECT_NAME].object.primitives[0].dimensions;

    // extract radius and return diameter
    return dim[1] * 2;
}