/**
 * @file panda.cpp
 * @author Enrico Sgarbanti
 * @brief panda implementations
 * @version 0.1
 * @date 20-02-2020
 *
 * @copyright Copyright (c) 2020 by Enrico Sgarbanti. License GPLv3.
 *
 */
#include "panda.hpp"



//#############################################################################
// PRIVATE IMPREMENTATIONS
double get_size_object(
    moveit::planning_interface::PlanningSceneInterfacePtr &planning_scene_ptr,
    const std::string &OBJECT_NAME) {
    // Get the attach objects
    auto attach_objects = planning_scene_ptr->getAttachedObjects();

    // Get dimensions object
    auto &dim = attach_objects[OBJECT_NAME].object.primitives[0].dimensions;

    // extract radius and return diameter
    return dim[1] * 2;
}



//#############################################################################
// PUBLIC METHODS IMPLEMENTATIONS #############################################
namespace robot {

Panda::Panda(const bool &GRIPPER_IS_ACTIVE) {
    // Init MoveGroupInterface with arm
    try {
        move_group_ptr_.reset(
            new moveit::planning_interface::MoveGroupInterface("panda_arm"));

    } catch (const std::runtime_error &e) {
        throw PCEXC::PandaArmException(
            "Panda::Panda()", "Impossible initialize MoveGroupInterface with "
                              "'panda_arm'");
    }

    // Init PlanningSceneInterface
    try {
        planning_scene_ptr_.reset(
            new moveit::planning_interface::PlanningSceneInterface);

    } catch (const std::runtime_error &e) {
        throw PCEXC::PandaArmException(
            "Panda::Panda()", "Impossible initialize PlanningSceneInterface");
    }

    // Set attributes
    setArmSpeed(defaults::ARM_SPEED);
    gripper_speed_ = defaults::GRIPPER_SPEED;
    gripper_is_active_ = GRIPPER_IS_ACTIVE;


    // Init Gripper
    if (gripper_is_active_) {
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
            throw PCEXC::PandaGripperException(
                "Panda::Panda()", "Impossible create "
                                  "action clients for franka_gripper");
        }
    }
}


geometry_msgs::Pose Panda::getCurrentPose() {
    return move_group_ptr_->getCurrentPose().pose;
}


void Panda::setArmSpeed(const float &SPEED) {
    move_group_ptr_->setMaxVelocityScalingFactor(SPEED);
}


void Panda::setGripperSpeed(const float &SPEED) {
    gripper_speed_ = SPEED;
}


void Panda::setScene(const moveit_msgs::PlanningScene &SCENE) {
    planning_scene_ptr_->applyPlanningScene(SCENE);
}


void Panda::resetScene() {
    moveit_msgs::PlanningScene scene_empty;
    planning_scene_ptr_->applyPlanningScene(scene_empty);
}


void Panda::moveJointsTo(const std::vector<double> &JOINTS) {
    // Set new joints values
    move_group_ptr_->setJointValueTarget(JOINTS);

    // Perform movement
    auto res = move_group_ptr_->move();

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PCEXC::PandaArmException("Panda::moveJointsTo()", "move()",
                                       "Failure");
}


void Panda::moveJointRad(const int &JOINT, const double &VAL) {
    if (JOINT > 7 || JOINT < 1)
        throw PCEXC::PandaArmException("Panda::moveJointRad()",
                                       "Joint invalid: joint_" +
                                           std::to_string(JOINT));

    auto joints_state = move_group_ptr_->getCurrentJointValues();

    // Update value of specified join
    // Note: joints_state start from 0 not 1
    joints_state[JOINT - 1] += VAL;

    // Perform movement
    moveJointsTo(joints_state);
}


void Panda::moveJointDeg(const int &JOINT, const double &VAL) {
    double rad_val = VAL * M_PI / 180.0;

    // Perform movement
    moveJointRad(JOINT, rad_val);
}


void Panda::moveToReadyPose() {
    // Perform movement
    moveJointsTo(config::READY_JOINTS);
}


void Panda::moveToPose(const geometry_msgs::Pose &POSE) {
    // Set the target Pose
    move_group_ptr_->setPoseTarget(POSE);

    // Perform movement
    auto res = move_group_ptr_->move();

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PCEXC::PandaArmException("Panda::moveToPose()", "move()",
                                       "Failure");
}


void Panda::cartesianMovement(const std::vector<geometry_msgs::Pose> &WAYPOINTS,
                              const double &STEP,
                              const double &JUMP_THRESHOLD) {
    moveit_msgs::RobotTrajectory trajectory;
    double progress_percentage = move_group_ptr_->computeCartesianPath(
        WAYPOINTS, STEP, JUMP_THRESHOLD, trajectory);

    // Check Errors
    if (progress_percentage == -1)
        throw PCEXC::PandaArmException("Panda::moveInCartesian()",
                                       "computeCartesianPath()", "Failure");

    // Abort if the progress percentage is not 100%
    if (progress_percentage != 1)
        throw PCEXC::PandaArmException(
            "Panda::moveInCartesian()", "computeCartesianPath()", "Failure:",
            "Only " + std::to_string(progress_percentage * 100) +
                "% completed");

    // Perform movement
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto res = move_group_ptr_->execute(plan);

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PCEXC::PandaArmException("Panda::cartesianMovement()",
                                       "execute()", "Failure");
}


void Panda::cartesianMovement(const geometry_msgs::Pose &POSE,
                              const double &STEP,
                              const double &JUMP_THRESHOLD) {
    std::vector<geometry_msgs::Pose> waypoint;
    waypoint.push_back(POSE);
    cartesianMovement(waypoint, STEP, JUMP_THRESHOLD);
}


void Panda::pick(const geometry_msgs::Pose &POSE,
                 const geometry_msgs::Vector3 &PRE_GRASP_APPROCH,
                 const double &GRASP_WIDTH, const double &GRASP_FORCE,
                 const double &GRASP_EPSILON_INNER,
                 const double &GRASP_EPSILON_OUTER, const double &STEP,
                 const double &JUMP_THRESHOLD) {
    try {
        // Open gripper
        gripperMove(config::GRIPPER_MAX_WIDTH);

        // Get pre-grasp-approch pose
        auto pre_pose = POSE;
        pre_pose.position.x += PRE_GRASP_APPROCH.x;
        pre_pose.position.y += PRE_GRASP_APPROCH.y;
        pre_pose.position.z += PRE_GRASP_APPROCH.z;

        // Move to pre-grasp-approch pose
        moveToPose(pre_pose);

        // Move to pose
        cartesianMovement(POSE, STEP, JUMP_THRESHOLD);

        // Close gripper
        gripperGrasp(GRASP_WIDTH, GRASP_FORCE, GRASP_EPSILON_INNER,
                     GRASP_EPSILON_OUTER);

    } catch (PCEXC::PandaArmException &e) {
        throw PCEXC::PandaArmException("Panda::pick()", std::string(e.what()));

    } catch (PCEXC::PandaGripperException &e) {
        throw PCEXC::PandaGripperException("Panda::pick()",
                                           std::string(e.what()));
    }
}


void Panda::pick(const geometry_msgs::Pose &POSE,
                 const std::string &OBJECT_NAME, const double &GRASP_FORCE,
                 const double &GRASP_EPSILON_INNER,
                 const double &GRASP_EPSILON_OUTER) {
    try {
        // Open gripper
        gripperMove(config::GRIPPER_MAX_WIDTH);

        // Move arm
        moveToPose(POSE);

        // Check if object exist
        bool exist = false;
        for (auto e : planning_scene_ptr_->getObjects()) {
            if (e.first == OBJECT_NAME)
                exist = true;
        }

        // Exit if object not exist
        if (!exist)
            throw PCEXC::PandaArmException("Not exists the object: " +
                                           OBJECT_NAME);

        // Attach object
        move_group_ptr_->attachObject(OBJECT_NAME,
                                      move_group_ptr_->getEndEffectorLink());

        // Get size object
        double size = get_size_object(planning_scene_ptr_, OBJECT_NAME);

        // Close gripper
        gripperGrasp(size, GRASP_FORCE, GRASP_EPSILON_INNER,
                     GRASP_EPSILON_OUTER);

    } catch (PCEXC::PandaArmException &e) {
        throw PCEXC::PandaArmException("Panda::pick()", std::string(e.what()));

    } catch (PCEXC::PandaGripperException &e) {
        throw PCEXC::PandaGripperException("Panda::pick()",
                                           std::string(e.what()));
    }
}


void Panda::place(const geometry_msgs::Pose &POSE,
                  const geometry_msgs::Vector3 &POST_GRASP_RETREAT,
                  const geometry_msgs::Vector3 &POST_PLACE_RETREAT,
                  const double &STEP, const double &JUMP_THRESHOLD) {
    try {
        // Get post-grasp-retrait pose
        auto PGR_pose = getCurrentPose();
        PGR_pose.position.x += POST_GRASP_RETREAT.x;
        PGR_pose.position.y += POST_GRASP_RETREAT.y;
        PGR_pose.position.z += POST_GRASP_RETREAT.z;

        // Move to post-grasp-retrait pose
        cartesianMovement(PGR_pose, STEP, JUMP_THRESHOLD);

        // Move arm
        moveToPose(POSE);

        // Open gripper
        gripperMove(config::GRIPPER_MAX_WIDTH);

        // Get post-place-retrait pose
        auto PPR_pose = getCurrentPose();
        PPR_pose.position.x += POST_PLACE_RETREAT.x;
        PPR_pose.position.y += POST_PLACE_RETREAT.y;
        PPR_pose.position.z += POST_PLACE_RETREAT.z;

        // Move to post-place-retrait pose
        cartesianMovement(PPR_pose, STEP, JUMP_THRESHOLD);

    } catch (PCEXC::PandaArmException &e) {
        throw PCEXC::PandaArmException("Panda::place()", std::string(e.what()));

    } catch (PCEXC::PandaGripperException &e) {
        throw PCEXC::PandaGripperException("Panda::place()",
                                           std::string(e.what()));
    }
}


void Panda::place(const geometry_msgs::Pose &POSE) {
    try {
        // Move arm
        moveToPose(POSE);

        // Detach objects
        move_group_ptr_->detachObject(move_group_ptr_->getEndEffectorLink());

        // Open gripper
        gripperMove(config::GRIPPER_MAX_WIDTH);

    } catch (PCEXC::PandaArmException &e) {
        throw PCEXC::PandaArmException("Panda::place()", std::string(e.what()));

    } catch (PCEXC::PandaGripperException &e) {
        throw PCEXC::PandaGripperException("Panda::place()",
                                           std::string(e.what()));
    }
}


void Panda::gripperHoming() {
    if (gripper_is_active_) {
        // Create homing goal
        auto goal = franka_gripper::HomingGoal();
        gripper_homing_client_ptr_->sendGoal(goal);

        // Wait for result
        if (!gripper_homing_client_ptr_->waitForResult()) {
            throw PCEXC::PandaGripperException("Panda::gripperHoming()",
                                               "waitForResult()", "Timeout");
        }
    }
}


void Panda::gripperMove(const double &WIDTH) {
    if (gripper_is_active_) {
        // Create move goal
        auto goal = franka_gripper::MoveGoal();
        goal.width = WIDTH;
        goal.speed = gripper_speed_;
        gripper_move_client_ptr_->sendGoal(goal);

        // Wait for result
        if (!gripper_move_client_ptr_->waitForResult()) {
            throw PCEXC::PandaGripperException("Panda::gripperMove()",
                                               "waitForResult()", "Timeout");
        }
    }
}


void Panda::gripperGrasp(const double &WIDTH, const double &FORCE,
                         const double &EPSILON_INNER,
                         const double &EPSILON_OUTER) {
    if (gripper_is_active_) {
        // Create move goal
        auto goal = franka_gripper::GraspGoal();
        goal.width = WIDTH;
        goal.speed = gripper_speed_;
        goal.force = FORCE;
        goal.epsilon.inner = EPSILON_INNER;
        goal.epsilon.outer = EPSILON_OUTER;
        gripper_grasp_client_ptr_->sendGoal(goal);

        // Wait for result
        if (!gripper_grasp_client_ptr_->waitForResult()) {
            throw PCEXC::PandaGripperException("Panda::gripperGrasp()",
                                               "waitForResult()", "Timeout");
        }
    }
}

}  // namespace robot