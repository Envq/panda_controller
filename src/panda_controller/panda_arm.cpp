// Custom
#include "panda_controller/panda_arm.hpp"
#define M_PI 3.14159265358979323846


// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



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


void PandaArm::setMaxVelocityScalingFactor(const float VELOCITY_FACTOR) {
    _arm_ptr->setMaxVelocityScalingFactor(VELOCITY_FACTOR);
}


std::vector<double> PandaArm::getJoints() {
    return _arm_ptr->getCurrentJointValues();
}


void PandaArm::moveToJoints(const std::vector<double> &JOINTS,
                            const bool ADJUST_IN_BOUNDS) {
    // Set new joints values
    bool in_bounds = _arm_ptr->setJointValueTarget(JOINTS);

    // Errors check
    if (!ADJUST_IN_BOUNDS && !in_bounds)
        throw PandaArmErr("moveToJoints()", "setJointValueTarget()",
                          "some joints have values beyond the limits");

    // Perform movement
    auto res = _arm_ptr->move();

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PandaArmErr("moveToJoints()", "move()", "Failure");
}


void PandaArm::moveToReady() {
    try {
        moveToJoints(panda_arm::READY_JOINTS);
    } catch (const PandaArmErr &err) {
        throw PandaArmErr("moveToReady()", err.what());
    }
}


geometry_msgs::Pose PandaArm::getPose() {
    return _arm_ptr->getCurrentPose().pose;
}


void PandaArm::moveToPose(const geometry_msgs::Pose &POSE) {
    // Set the target Pose
    _arm_ptr->setPoseTarget(POSE);

    // Perform movement
    auto res = _arm_ptr->move();

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PandaArmErr("moveToPose()", "move()", "Failure");
}


void PandaArm::relativeMovePos(const double X, const double Y, const double Z) {
    try {
        // Get current pose
        auto pose = getPose();

        // Update pose
        pose.position.x += X;
        pose.position.y += Y;
        pose.position.z += Z;

        // Move to target
        linearMove(pose);
    } catch (const PandaArmErr &err) {
        throw PandaArmErr("moveRelPosition()", err.what());
    }
}


void PandaArm::relativeMoveRPY(const double ROLL, const double PITCH,
                               const double YAW) {
    try {
        // Get current pose
        auto pose = getPose();
        tf2::Quaternion quat1;
        tf2::convert(pose.orientation, quat1);

        // Get quaternion
        tf2::Quaternion quat2;
        quat2.setRPY(ROLL * M_PI / 180.0, PITCH * M_PI / 180.0,
                     YAW * M_PI / 180.0);

        // Update pose orientation
        pose.orientation = tf2::toMsg((quat1 * quat2).normalize());

        // Move to target
        linearMove(pose);
    } catch (const PandaArmErr &err) {
        throw PandaArmErr("moveRelPosition()", err.what());
    }
}


void PandaArm::waypointsMove(const std::vector<geometry_msgs::Pose> &WAYPOINTS,
                             const double EEF_STEP,
                             const double JUMP_THRESHOLD) {
    moveit_msgs::RobotTrajectory trajectory;
    double progress_percentage = _arm_ptr->computeCartesianPath(
        WAYPOINTS, EEF_STEP, JUMP_THRESHOLD, trajectory);

    // Check Errors
    if (progress_percentage == -1)
        throw PandaArmErr("waypointsMove()", "computeCartesianPath()",
                          "Failure");

    // Abort if the progress percentage is not 100%
    if (progress_percentage != 1)
        throw PandaArmErr("waypointsMove()", "computeCartesianPath()",
                          "Failure:",
                          "Only " + std::to_string(progress_percentage * 100) +
                              "% completed");

    // Perform movement
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    auto res = _arm_ptr->execute(plan);

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PandaArmErr("waypointsMove()", "execute()", "Failure");
}


void PandaArm::linearMove(const geometry_msgs::Pose &POSE,
                          const double EEF_STEP, const double JUMP_THRESHOLD) {
    std::vector<geometry_msgs::Pose> waypoint;
    waypoint.push_back(POSE);
    try {
        waypointsMove(waypoint, EEF_STEP, JUMP_THRESHOLD);
    } catch (const PandaArmErr &err) {
        throw PandaArmErr("linearMove()", err.what());
    }
}


void PandaArm::savePose(const std::string &POSE_NAME) {
    auto current_pose = getPose();
    save_pose(current_pose, POSE_NAME);
}


void PandaArm::moveToPose(const std::string &POSE_NAME) {
    auto target_pose = load_pose(POSE_NAME);
    moveToPose(target_pose);
}

}  // namespace panda_controller