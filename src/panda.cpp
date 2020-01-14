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
        move_group_ptr_.reset(
            new moveit::planning_interface::MoveGroupInterface("panda_arm"));

    } catch (const std::runtime_error &e) {
        throw PCEXC::panda_error(
            "Panda::Panda()" + PCEXC::DIVISOR +
            "Impossible initialize MoveGroupInterface with "
            "'panda_arm'");
    }

    // Init PlanningSceneInterface
    try {
        planning_scene_ptr_.reset(
            new moveit::planning_interface::PlanningSceneInterface);

    } catch (const std::runtime_error &e) {
        throw PCEXC::panda_error(
            "Panda::Panda()" + PCEXC::DIVISOR +
            "Impossible initialize PlanningSceneInterface");
    }

    // Init Gripper
    try {
        // Create action client for homing
        gripper_homing_client_ptr_.reset(
            new GripperHomingClient("franka_gripper/homing", true));

        // Create action client for move
        gripper_move_client_ptr_.reset(
            new GripperMoveClient("franka_gripper/move", true));

        // Create action client for grasp
        gripper_grasp_client_ptr_.reset(
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
    return move_group_ptr_->getCurrentPose().pose;
}


void Panda::setArmSpeed(const float &SPEED) {
    move_group_ptr_->setMaxVelocityScalingFactor(SPEED);
}


void Panda::setScene(const moveit_msgs::PlanningScene &SCENE) {
    planning_scene_ptr_->applyPlanningScene(SCENE);
}


void Panda::resetScene() {
    moveit_msgs::PlanningScene scene_empty;
    planning_scene_ptr_->applyPlanningScene(scene_empty);
}


void Panda::moveToPosition(const geometry_msgs::Pose &POSE,
                           const bool &PLAN_ONLY) {
    // Set the target Pose
    move_group_ptr_->setPoseTarget(POSE);

    // Perform the planning
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto res = move_group_ptr_->plan(plan);

    // State of planning check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PCEXC::panda_arm_error("Panda::moveToPosition()" +
                                     PCEXC::DIVISOR + "plan()" +
                                     PCEXC::DIVISOR + "failure");

    // Execute the move
    if (!PLAN_ONLY)
        move_group_ptr_->move();
}


void Panda::cartesianMovement(const std::vector<geometry_msgs::Pose> &WAYPOINTS,
                              const double &STEP, const double &JUMP_THRESHOLD,
                              const bool &PLAN_ONLY) {
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group_ptr_->computeCartesianPath(
        WAYPOINTS, STEP, JUMP_THRESHOLD, trajectory);

    // Check Errors
    if (fraction == -1)
        throw PCEXC::panda_arm_error("Panda::moveInCartesian()" +
                                     PCEXC::DIVISOR + "computeCartesianPath()" +
                                     PCEXC::DIVISOR + "failure");
    if (fraction != 1)
        throw PCEXC::panda_arm_error(
            "Panda::moveInCartesian()" + PCEXC::DIVISOR +
            "computeCartesianPath()" + PCEXC::DIVISOR + "failure: only " +
            std::to_string(fraction * 100) + "% completed");

    // Execute the move
    if (!PLAN_ONLY) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group_ptr_->execute(plan);
    }
}


void Panda::cartesianMovement(const geometry_msgs::Pose &POSE,
                              const double &STEP, const double &JUMP_THRESHOLD,
                              const bool &PLAN_ONLY) {
    std::vector<geometry_msgs::Pose> waypoint;
    waypoint.push_back(POSE);
    cartesianMovement(waypoint, STEP, JUMP_THRESHOLD, PLAN_ONLY);
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

    } catch (PCEXC::panda_arm_error &e) {
        throw PCEXC::panda_arm_error("Panda::pick()" + PCEXC::DIVISOR + "" +
                                     std::string(e.what()));

    } catch (PCEXC::panda_gripper_error &e) {
        throw PCEXC::panda_gripper_error("Panda::place() >>" +
                                         std::string(e.what()));
    }
}


void Panda::pick(const geometry_msgs::Pose &POSE, const float &WIDTH,
                 const geometry_msgs::Vector3 &PRE_GRASP_APPROCH,
                 const bool &PLAN_ONLY) {
    try {
        // Open gripper
        gripperMove(robot::GRIPPER_MAX_WIDTH);

        // Get pre-grasp-approch pose
        auto pre_pose = POSE;
        pre_pose.position.x += PRE_GRASP_APPROCH.x;
        pre_pose.position.y += PRE_GRASP_APPROCH.y;
        pre_pose.position.z += PRE_GRASP_APPROCH.z;

        // Move to pre-grasp-approch pose
        moveToPosition(pre_pose);

        // Move to pose
        cartesianMovement(POSE);

        // Close gripper
        gripperGrasp(WIDTH);

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

    } catch (PCEXC::panda_arm_error &e) {
        throw PCEXC::panda_arm_error("Panda::place() >>" +
                                     std::string(e.what()));

    } catch (PCEXC::panda_gripper_error &e) {
        throw PCEXC::panda_gripper_error("Panda::place() >>" +
                                         std::string(e.what()));
    }
}


void Panda::place(const geometry_msgs::Pose &POSE,
                  const geometry_msgs::Vector3 &POST_GRASP_RETREAT,
                  const geometry_msgs::Vector3 &POST_PLACE_RETREAT,
                  const bool &PLAN_ONLY) {
    try {
        // Get post-grasp-retrait pose
        auto PGR_pose = getCurrentPose();
        PGR_pose.position.x += POST_GRASP_RETREAT.x;
        PGR_pose.position.y += POST_GRASP_RETREAT.y;
        PGR_pose.position.z += POST_GRASP_RETREAT.z;

        // Move to post-grasp-retrait pose
        cartesianMovement(PGR_pose);

        // Move arm
        moveToPosition(POSE);

        // Open gripper
        gripperMove(robot::GRIPPER_MAX_WIDTH);

        // Get post-place-retrait pose
        auto PPR_pose = getCurrentPose();
        PPR_pose.position.x += POST_PLACE_RETREAT.x;
        PPR_pose.position.y += POST_PLACE_RETREAT.y;
        PPR_pose.position.z += POST_PLACE_RETREAT.z;

        // Move to post-place-retrait pose
        cartesianMovement(PPR_pose);

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
    if (!gripper_homing_client_ptr_->isServerConnected()) {
        throw PCEXC::panda_gripper_error(
            "Panda::gripperHoming()" + PCEXC::DIVISOR + "isServerConnected()" +
            PCEXC::DIVISOR + "Server is not connected");
    }

    // Wait for action server
    if (!gripper_homing_client_ptr_->waitForServer()) {
        throw PCEXC::panda_gripper_error("Panda::gripperHoming()" +
                                         PCEXC::DIVISOR + "waitForServer()" +
                                         PCEXC::DIVISOR + "Timeout");
    }

    // Create homing goal
    auto goal = franka_gripper::HomingGoal();
    gripper_homing_client_ptr_->sendGoal(goal);

    // Wait for result
    if (!gripper_homing_client_ptr_->waitForResult()) {
        throw PCEXC::panda_gripper_error("Panda::gripperHoming()" +
                                         PCEXC::DIVISOR + "waitForResult()" +
                                         PCEXC::DIVISOR + "Timeout");
    }
}


void Panda::gripperMove(const float &WIDTH, const float &SPEED) {
    // Check if server is connected
    if (!gripper_homing_client_ptr_->isServerConnected()) {
        throw PCEXC::panda_gripper_error(
            "Panda::gripperHoming()" + PCEXC::DIVISOR + "isServerConnected()" +
            PCEXC::DIVISOR + "Server is not connected");
    }

    // Wait for action server
    if (!gripper_move_client_ptr_->waitForServer()) {
        throw PCEXC::panda_gripper_error("Panda::gripperMove()" +
                                         PCEXC::DIVISOR + "waitForServer()" +
                                         PCEXC::DIVISOR + "Timeout");
    }

    // Create move goal
    auto goal = franka_gripper::MoveGoal();
    goal.width = WIDTH;
    goal.speed = SPEED;
    gripper_move_client_ptr_->sendGoal(goal);

    // Wait for result
    if (!gripper_move_client_ptr_->waitForResult()) {
        throw PCEXC::panda_gripper_error("Panda::gripperMove()" +
                                         PCEXC::DIVISOR + "waitForResult()" +
                                         PCEXC::DIVISOR + "Timeout");
    }
}


void Panda::gripperGrasp(const float &WIDTH, const float &SPEED,
                         const float &FORCE, const float &EPSILON_INNER,
                         const float &EPSILON_OUTER) {
    // Check if server is connected
    if (!gripper_homing_client_ptr_->isServerConnected()) {
        throw PCEXC::panda_gripper_error(
            "Panda::gripperHoming()" + PCEXC::DIVISOR + "isServerConnected()" +
            PCEXC::DIVISOR + "Server is not connected");
    }

    // Wait for action server
    if (!gripper_grasp_client_ptr_->waitForServer()) {
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
    gripper_grasp_client_ptr_->sendGoal(goal);

    // Wait for result
    if (!gripper_grasp_client_ptr_->waitForResult()) {
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