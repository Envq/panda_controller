// Custom
#include "panda_controller/panda_gripper.hpp"



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CLASSES ====================================================================
PandaGripper::PandaGripper(const bool REAL_ROBOT) {
    // Initialize
    _real_robot = REAL_ROBOT;
    _current_width = -1;  // use homing for override it
    _timeout = 0;         // inf

    if (REAL_ROBOT) {
        try {
            // Create action client for homing
            _homing_client_ptr.reset(
                new GripperHomingClient("franka_gripper/homing", true));

            // Create action client for move
            _move_client_ptr.reset(
                new GripperMoveClient("franka_gripper/move", true));

            // Create action client for grasp
            _grasp_client_ptr.reset(
                new GripperGraspClient("franka_gripper/grasp", true));

        } catch (const std::runtime_error &err) {
            throw PandaGripperErr(
                "PandaGripper()",
                "Impossible create action clients for franka_gripper");
        }

    } else {
        // Init MoveGroupInterface with hand
        std::string move_group_name = "hand";
        try {
            _hand_ptr.reset(new moveit::planning_interface::MoveGroupInterface(
                move_group_name));

        } catch (const std::runtime_error &err) {
            throw PandaGripperErr(
                "PandaGripper()",
                "Impossible initialize MoveGroupInterface with '" +
                    move_group_name + "'");
        }
    }
}


double PandaGripper::_normalize(const double VAL, const double MIN,
                                const double MAX) {
    if (VAL < MIN)
        return MIN;
    if (VAL > MAX)
        return MAX;
    return VAL;
}


void PandaGripper::_moveHand(const double WIDTH_NORMALIZED) {
    // Prepare fingers
    std::vector<double> fingers;
    fingers.push_back(WIDTH_NORMALIZED / 2.0);
    fingers.push_back(WIDTH_NORMALIZED / 2.0);
    _hand_ptr->setJointValueTarget(fingers);

    // Perform movement
    auto res = _hand_ptr->move();

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PandaGripperErr("_moveHand()", "move()", "Failure");
}


void PandaGripper::setTiming(const double TIMEOUT) {
    _timeout = TIMEOUT;
}


double PandaGripper::getWidth() {
    return _current_width;
}


void PandaGripper::homing() {
    if (_real_robot) {
        // Create homing goal
        auto goal = franka_gripper::HomingGoal();
        _homing_client_ptr->sendGoal(goal);

        // Wait for result
        if (!_homing_client_ptr->waitForResult(ros::Duration(_timeout)))
            throw PandaGripperErr("homing()", "waitForResult()", "Timeout");
    }

    // Update current width
    _current_width = OPEN;
}


void PandaGripper::move(const double WIDTH, const double SPEED) {
    double width_normalized =
        _normalize(WIDTH, panda_gripper::MIN_WIDTH, panda_gripper::MAX_WIDTH);

    if (_real_robot) {
        // Create move goal
        auto goal = franka_gripper::MoveGoal();
        goal.width = width_normalized;
        goal.speed = SPEED;
        _move_client_ptr->sendGoal(goal);

        // Wait for result
        if (!_move_client_ptr->waitForResult(ros::Duration(_timeout)))
            throw PandaGripperErr("move()", "waitForResult()", "Timeout");

    } else
        _moveHand(width_normalized);

    // Update current width
    _current_width = width_normalized;
}


void PandaGripper::grasp(const double WIDTH, const double SPEED,
                         const double FORCE, const double EPSILON_INNER,
                         const double EPSILON_OUTER) {
    double width_normalized =
        _normalize(WIDTH, panda_gripper::MIN_WIDTH, panda_gripper::MAX_WIDTH);
    double force_normalized =
        _normalize(WIDTH, panda_gripper::MIN_FORCE, panda_gripper::MAX_FORCE);

    if (_real_robot) {
        // Create move goal
        auto goal = franka_gripper::GraspGoal();
        goal.width = width_normalized;
        goal.speed = SPEED;
        goal.force = force_normalized;
        goal.epsilon.inner = EPSILON_INNER;
        goal.epsilon.outer = EPSILON_OUTER;
        _grasp_client_ptr->sendGoal(goal);

        // Wait for result
        if (!_grasp_client_ptr->waitForResult(ros::Duration(_timeout)))
            throw PandaGripperErr("grasp()", "waitForResult()", "Timeout");

    } else
        _moveHand(width_normalized);

    // Update current width
    _current_width = width_normalized;
}

}  // namespace panda_controller