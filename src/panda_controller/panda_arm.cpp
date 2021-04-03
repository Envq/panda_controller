// Custom
#include "panda_controller/panda_arm.hpp"
#define M_PI 3.14159265358979323846


// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CLASSES ====================================================================
PandaArm::PandaArm(const float DELAY) {
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

    // Init TF2
    _tf_buffer_ptr.reset(new tf2_ros::Buffer());
    _tf_listener_ptr.reset(new tf2_ros::TransformListener(*_tf_buffer_ptr));
    ros::Duration(DELAY).sleep();

    try {
        // tcp -> flange transformation
        auto tcp_to_flange_msg =
            _tf_buffer_ptr->lookupTransform("tcp", "panda_link8", ros::Time(0));

        // Convert from geometry_msgs::TransformStamped -> tf2::Transform
        _tcp_to_flange = tf2::Transform(
            tf2::Quaternion{tcp_to_flange_msg.transform.rotation.x,
                            tcp_to_flange_msg.transform.rotation.y,
                            tcp_to_flange_msg.transform.rotation.z,
                            tcp_to_flange_msg.transform.rotation.w},
            tf2::Vector3{tcp_to_flange_msg.transform.translation.x,
                         tcp_to_flange_msg.transform.translation.y,
                         tcp_to_flange_msg.transform.translation.z});

    } catch (tf2::TransformException &err) {
        throw PandaArmErr("PandaArm()", "lookupTransform()",
                          "transformation (tcp -> panda_link8) error");
    };
}


geometry_msgs::Pose
PandaArm::_getFlangeFromTCP(const geometry_msgs::Pose &TCP_POSE) {
    // get TCP trasform
    tf2::Transform world_to_tcp;
    tf2::fromMsg(TCP_POSE, world_to_tcp);

    // Calculate the Flange transform
    tf2::Transform world_to_flange;
    world_to_flange = world_to_tcp * _tcp_to_flange;

    // return the Flange pose
    geometry_msgs::Pose flange_pose;
    tf2::toMsg(world_to_flange, flange_pose);
    return flange_pose;
}


moveit::planning_interface::MoveGroupInterfacePtr PandaArm::getMoveGroup() {
    return _arm_ptr;
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
    try {
        geometry_msgs::TransformStamped tf =
            _tf_buffer_ptr->lookupTransform("world", "tcp", ros::Time(0));

        geometry_msgs::Pose current_pose;
        current_pose.position.x = tf.transform.translation.x;
        current_pose.position.y = tf.transform.translation.y;
        current_pose.position.z = tf.transform.translation.z;
        current_pose.orientation.x = tf.transform.rotation.x;
        current_pose.orientation.y = tf.transform.rotation.y;
        current_pose.orientation.z = tf.transform.rotation.z;
        current_pose.orientation.w = tf.transform.rotation.w;

        return current_pose;

    } catch (tf2::TransformException &err) {
        throw PandaArmErr("getPose()", "lookupTransform()",
                          "transformation error");
    }
}


void PandaArm::moveToPose(const geometry_msgs::Pose &POSE) {
    // Get flange pose
    geometry_msgs::Pose target;
    try {
        target = _getFlangeFromTCP(POSE);
    } catch (const PandaArmErr &err) {
        throw PandaArmErr("moveToPose()", err.what());
    }

    // Set the target Pose
    _arm_ptr->setPoseTarget(target);

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
    // Convert TCP waypoints into Flange waypoints
    std::vector<geometry_msgs::Pose> targets;
    try {
        for (const auto point : WAYPOINTS) {
            targets.push_back(_getFlangeFromTCP(point));
        }
    } catch (const PandaArmErr &err) {
        throw PandaArmErr("moveToPose()", err.what());
    }

    // Generate trajectory
    moveit_msgs::RobotTrajectory trajectory;
    double progress_percentage = _arm_ptr->computeCartesianPath(
        targets, EEF_STEP, JUMP_THRESHOLD, trajectory);

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
    try {
        save_pose(current_pose, POSE_NAME);
    } catch (const PandaArmErr &err) {
        throw DataManagerErr("savePose()", err.what());
    }
}


void PandaArm::moveToPose(const std::string &POSE_NAME) {
    geometry_msgs::Pose target;
    try {
        target = load_pose(POSE_NAME);
    } catch (const PandaArmErr &err) {
        throw DataManagerErr("moveToPose()", err.what());
    }
    moveToPose(target);
}

}  // namespace panda_controller