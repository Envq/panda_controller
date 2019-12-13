#include "panda.hpp"



//#############################################################################
// PRIVATE FUNCTIONS AND ENUMERATIONS
// TODO: now get r of cylinder. Extend it
float get_size_object(
    moveit::planning_interface::PlanningSceneInterfacePtr &planning_scene_ptr,
    const std::string &OBJECT_NAME);



//#############################################################################
// PUBLIC METHODS IMPLEMENTATIONS
namespace robot {

Panda::Panda() {
    // Init MoveGroupInterface with arm
    try {
        arm_group_ptr.reset(
            new moveit::planning_interface::MoveGroupInterface("panda_arm"));

    } catch (const std::runtime_error &e) {
        throw my_exceptions::panda_error(
            "Panda::Panda() >> Impossible initialize MoveGroupInterface with "
            "'panda_arm'");
    }

    // Init PlanningSceneInterface
    try {
        planning_scene_ptr.reset(
            new moveit::planning_interface::PlanningSceneInterface);

    } catch (const std::runtime_error &e) {
        throw my_exceptions::panda_error(
            "Panda::Panda() >> Impossible initialize PlanningSceneInterface");
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
        throw my_exceptions::panda_error("Panda::Panda() >> Impossible create "
                                         "action clients for franka_gripper");
    }

    // Set default arm speed
    setArmSpeed(robot::DEFAULT_ARM_SPEED);

    // Homing gripper
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
        throw my_exceptions::panda_error(
            "Panda::moveToPosition() >> Planning failure");

    // Execute the move
    if (!PLAN_ONLY)
        arm_group_ptr->move();
}


void Panda::pick(const geometry_msgs::Pose &POSE,
                 const std::string &OBJECT_NAME, const bool &PLAN_ONLY) {
    try {
        // Open gripper
        gripperMove(robot::GRIPPER_MAX_WIDTH);

        // Move arm
        moveToPosition(POSE);

        // Attach object
        arm_group_ptr->attachObject(OBJECT_NAME);

        // Get size object
        float size = get_size_object(planning_scene_ptr, OBJECT_NAME);

        // Close gripper
        gripperGrasp(size);

    } catch (my_exceptions::panda_error &e) {
        throw my_exceptions::panda_error("Panda::pick() >> " +
                                         std::string(e.what()));
    }
}


void Panda::place(const geometry_msgs::Pose &POSE,
                  const std::string &OBJECT_NAME, const bool &PLAN_ONLY) {
    try {
        // Move arm
        moveToPosition(POSE);

        // Open gripper
        gripperMove(robot::GRIPPER_MAX_WIDTH);

        // Detach object
        arm_group_ptr->detachObject(OBJECT_NAME);

    } catch (my_exceptions::panda_error &e) {
        throw my_exceptions::panda_error("Panda::place() >>" +
                                         std::string(e.what()));
    }
}


void Panda::gripperHoming() {
    // Wait for action server
    if (!gripper_homing_client_ptr->waitForServer()) {
        throw my_exceptions::panda_error(
            "Panda::gripperHoming() >> waitForServer() >> Timeout");
    }

    // Create homing goal
    auto goal = franka_gripper::HomingGoal();
    gripper_homing_client_ptr->sendGoal(goal);

    // Wait for result
    if (!gripper_homing_client_ptr->waitForResult()) {
        throw my_exceptions::panda_error(
            "Panda::gripperHoming() >> waitForResult() >> Timeout");
    }
}


void Panda::gripperMove(const float &WIDTH, const float &SPEED) {
    // Wait for action server
    if (!gripper_move_client_ptr->waitForServer()) {
        throw my_exceptions::panda_error(
            "Panda::gripperMove() >> waitForServer() >> Timeout");
    }

    // Create move goal
    auto goal = franka_gripper::MoveGoal();
    goal.width = WIDTH;
    goal.speed = SPEED;
    gripper_move_client_ptr->sendGoal(goal);

    // Wait for result
    if (!gripper_move_client_ptr->waitForResult()) {
        throw my_exceptions::panda_error(
            "Panda::gripperMove() >> waitForResult() >> Timeout");
    }
}


void Panda::gripperGrasp(const float &WIDTH, const float &SPEED, const float &FORCE,
                  const float &EPSILON_INNER, const float &EPSILON_OUTER) {
    // Wait for action server
    if (!gripper_grasp_client_ptr->waitForServer()) {
        throw my_exceptions::panda_error(
            "Panda::gripperGrasp() >> waitForServer() >> Timeout");
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
    if (!gripper_move_client_ptr->waitForResult()) {
        throw my_exceptions::panda_error(
            "Panda::gripperGrasp() >> waitForResult() >> Timeout");
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