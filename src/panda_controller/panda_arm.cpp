// Custom
#include "panda_controller/panda_arm.hpp"



// USING NAMESPACE ============================================================
using namespace panda_controller;



// CLASSES ====================================================================
PandaArm::PandaArm() {
    // Init MoveGroupInterface with arm
    std::string move_group_name = "panda_arm";
    try {
        _arm_ptr.reset(new moveit::planning_interface::MoveGroupInterface(
            move_group_name));

    } catch (const std::runtime_error &err) {
        throw PandaArmErr("PandaArm()",
                          "Impossible initialize MoveGroupInterface with '" +
                              move_group_name + "'");
    }
}


void PandaArm::setMaxVelocityScalingFactor(const float &VELOCITY_FACTOR) {
    _arm_ptr->setMaxVelocityScalingFactor(VELOCITY_FACTOR);
}


std::vector<double> PandaArm::getArmJoints() {
    return _arm_ptr->getCurrentJointValues();
}


void PandaArm::moveArmJoints(const std::vector<double> &JOINTS,
                             const bool &ADJUST_IN_BOUNDS) {
    // Set new joints values
    bool in_bounds = _arm_ptr->setJointValueTarget(JOINTS);

    // Errors check
    if (!ADJUST_IN_BOUNDS && !in_bounds)
        throw PandaArmErr("moveJointsTo()", "setJointValueTarget()",
                          "some joints have values beyond the limits");

    // Perform movement
    auto res = _arm_ptr->move();

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PandaArmErr("moveJointsTo()", "move()", "Failure");
}


void PandaArm::moveArmReady() {
    try {
        moveArmJoints(panda_arm::READY_JOINTS);
    } catch (const PandaArmErr &err) {
        throw PandaArmErr("moveArmReady()", err.what());
    }
}


geometry_msgs::Pose PandaArm::getArmPose() {
    return _arm_ptr->getCurrentPose().pose;
}


void PandaArm::moveArmPose(const geometry_msgs::Pose &POSE) {
    // Set the target Pose
    _arm_ptr->setPoseTarget(POSE);

    // Perform movement
    auto res = _arm_ptr->move();

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PandaArmErr("moveArmPose()", "move()", "Failure");
}


void PandaArm::cartesianMovement(
    const std::vector<geometry_msgs::Pose> &WAYPOINTS, const double &STEP,
    const double &JUMP_THRESHOLD) {
    moveit_msgs::RobotTrajectory trajectory;
    double progress_percentage = _arm_ptr->computeCartesianPath(
        WAYPOINTS, STEP, JUMP_THRESHOLD, trajectory);

    // Check Errors
    if (progress_percentage == -1)
        throw PandaArmErr("moveInCartesian()", "computeCartesianPath()",
                          "Failure");

    // Abort if the progress percentage is not 100%
    if (progress_percentage != 1)
        throw PandaArmErr("moveInCartesian()", "computeCartesianPath()",
                          "Failure:",
                          "Only " + std::to_string(progress_percentage * 100) +
                              "% completed");

    // Perform movement
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto res = _arm_ptr->execute(plan);

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PandaArmErr("cartesianMovement()", "execute()", "Failure");
}


void PandaArm::cartesianMovement(const geometry_msgs::Pose &POSE,
                                 const double &STEP,
                                 const double &JUMP_THRESHOLD) {
    std::vector<geometry_msgs::Pose> waypoint;
    waypoint.push_back(POSE);
    cartesianMovement(waypoint, STEP, JUMP_THRESHOLD);
}