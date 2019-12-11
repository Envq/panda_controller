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

    // Init MoveGroupInterface with hand
    try {
        hand_group_ptr.reset(
            new moveit::planning_interface::MoveGroupInterface("hand"));

    } catch (const std::runtime_error &e) {
        throw my_exceptions::panda_error(
            "Panda::Panda() >> Impossible initialize MoveGroupInterface with "
            "'hand'");
    }

    // Init PlanningSceneInterface
    try {
        planning_scene_ptr.reset(
            new moveit::planning_interface::PlanningSceneInterface);

    } catch (const std::runtime_error &e) {
        throw my_exceptions::panda_error(
            "Panda::Panda() >> Impossible initialize PlanningSceneInterface");
    }

    // set default speed
    speed = 1.0;
}


geometry_msgs::Pose Panda::getCurrentPose() {
    return arm_group_ptr->getCurrentPose().pose;
}


void Panda::setSpeed(const float &SPEED) {
    arm_group_ptr->setMaxVelocityScalingFactor(SPEED);
    speed = SPEED;
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

void Panda::moveGripper(const float &WIDTH, const bool &PLAN_ONLY) {
    // Value correctness check
    if (WIDTH > robot::GRIPPER_MAX_WIDTH || WIDTH < 0.0)
        throw my_exceptions::panda_error(
            "Panda::moveGripper() >> WIDTH must be between 0.00 and 0.04");

    // Set new joints value
    std::vector<double> gripper_joints;
    gripper_joints.push_back(WIDTH / 2.0);
    gripper_joints.push_back(WIDTH / 2.0);  // mimic

    // Set the new joints state
    hand_group_ptr->setJointValueTarget(gripper_joints);

    // Perform the planning
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = hand_group_ptr->plan(plan);

    // State of planning check
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw my_exceptions::panda_error(
            "Panda::moveGripper() >> Planning failure");

    // Execute the move
    if (!PLAN_ONLY)
        hand_group_ptr->move();
}


void Panda::pick(const geometry_msgs::Pose &POSE,
                 const std::string &OBJECT_NAME, const bool &PLAN_ONLY) {
    try {
        // Open gripper
        moveGripper(robot::GRIPPER_MAX_WIDTH);

        // Move arm
        moveToPosition(POSE);

        // Attach object
        arm_group_ptr->attachObject(OBJECT_NAME);

        // Get size object
        float size = get_size_object(planning_scene_ptr, OBJECT_NAME);

        // Close gripper
        moveGripper(size + robot::EPSILON_GRASP);

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
        moveGripper(GRIPPER_MAX_WIDTH);

        // Detach object
        arm_group_ptr->detachObject(OBJECT_NAME);

    } catch (my_exceptions::panda_error &e) {
        throw my_exceptions::panda_error("Panda::place() >>" +
                                         std::string(e.what()));
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