// Custom
#include "panda_controller/panda_gripper.hpp"



// USING NAMESPACE ============================================================
using namespace panda_controller;



// CLASSES ====================================================================
PandaGripper::PandaGripper(const bool &REAL_ROBOT) {
    // Initialize
    _curren_width = -1;
    _real_robot = REAL_ROBOT;

    if (REAL_ROBOT) {
        try {
            // Create action client for homing
            _gripper_homing_client =
                new GripperHomingClient("franka_gripper/homing", true);

            // Create action client for move
            _gripper_move_client =
                new GripperMoveClient("franka_gripper/move", true);

            // Create action client for grasp
            _gripper_grasp_client =
                new GripperGraspClient("franka_gripper/grasp", true);

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


void PandaGripper::_moveHand(const double &WIDTH) {
    // Prepare fingers
    std::vector<double> fingers;
    fingers.push_back(WIDTH / 2.0);
    fingers.push_back(WIDTH / 2.0);
    _hand_ptr->setJointValueTarget(fingers);

    // Perform movement
    auto res = _hand_ptr->move();

    // Errors check
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw PandaGripperErr("_moveHand()", "move()", "Failure");
}


void PandaGripper::homing() {
    if (_real_robot) {
        // Create homing goal
        auto goal = franka_gripper::HomingGoal();
        _gripper_homing_client->sendGoal(goal);

        // Wait for result
        if (!_gripper_homing_client->waitForResult()) {
            throw PandaGripperErr("homing()", "waitForResult()", "Timeout");
        }
    }
}


void PandaGripper::move(const double &WIDTH, const double &SPEED) {
    if (_real_robot) {
        // Create move goal
        auto goal = franka_gripper::MoveGoal();
        goal.width = WIDTH;
        goal.speed = SPEED;
        _gripper_move_client->sendGoal(goal);

        // Wait for result
        if (!_gripper_move_client->waitForResult()) {
            throw PandaGripperErr("move()", "waitForResult()", "Timeout");
        }
    } else {
        _moveHand(WIDTH);
    }
}


void PandaGripper::grasp(const double &WIDTH, const double &SPEED,
                         const double &FORCE, const double &EPSILON_INNER,
                         const double &EPSILON_OUTER) {
    if (_real_robot) {
        // Create move goal
        auto goal = franka_gripper::GraspGoal();
        goal.width = WIDTH;
        goal.speed = SPEED;
        goal.force = FORCE;
        goal.epsilon.inner = EPSILON_INNER;
        goal.epsilon.outer = EPSILON_OUTER;
        _gripper_grasp_client->sendGoal(goal);

        // Wait for result
        if (!_gripper_grasp_client->waitForResult()) {
            throw PandaGripperErr("grasp()", "waitForResult()", "Timeout");
        }
    } else {
        _moveHand(WIDTH);
    }
}