// Custom
#include "panda_controller/panda.hpp"



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CLASSES ====================================================================
Panda::Panda(const bool REAL_ROBOT, const float DELAY) {
    // Init PandaArm and PandaGripper
    try {
        _panda_arm_ptr.reset(new PandaArm(DELAY));
        _panda_gripper_ptr.reset(new PandaGripper(REAL_ROBOT));
        _panda_scene_ptr.reset(new PandaScene(DELAY));

    } catch (const std::runtime_error &err) {
        throw PandaErr("Panda()", err.what());
    }
}


std::shared_ptr<PandaArm> Panda::getArm() const {
    return _panda_arm_ptr;
}


std::shared_ptr<PandaGripper> Panda::getGripper() const {
    return _panda_gripper_ptr;
}


std::shared_ptr<PandaScene> Panda::getScene() const {
    return _panda_scene_ptr;
}


void Panda::pick(const geometry_msgs::Pose &PRE_GRASP_APPROCH,
                 const geometry_msgs::Pose &OBJECT, const double &GRASP_WIDTH,
                 const double &GRASP_FORCE, const double &GRASP_EPSILON_INNER,
                 const double &GRASP_EPSILON_OUTER, const double &EEF_STEP,
                 const double &JUMP_THRESHOLD) {
    try {
        // Open gripper
        _panda_gripper_ptr->move(PandaGripper::OPEN);

        // Move to pre-grasp-approch pose
        _panda_arm_ptr->moveToPose(PRE_GRASP_APPROCH);

        // Move to pose
        _panda_arm_ptr->linearMove(OBJECT, EEF_STEP, JUMP_THRESHOLD);

        // Close gripper
        _panda_gripper_ptr->grasp(GRASP_WIDTH, GRASP_FORCE, GRASP_EPSILON_INNER,
                                  GRASP_EPSILON_OUTER);

    } catch (PandaArmErr &e) {
        throw PandaErr("pick()", e.what());

    } catch (PandaGripperErr &e) {
        throw PandaErr("pick()", e.what());
    }
}


void Panda::place(const geometry_msgs::Pose &POST_GRASP_RETREAT,
                  const geometry_msgs::Pose &GOAL,
                  const geometry_msgs::Pose &POST_PLACE_RETREAT,
                  const double &EEF_STEP, const double &JUMP_THRESHOLD) {
    try {
        // Move to post-grasp-retrait pose
        _panda_arm_ptr->linearMove(POST_GRASP_RETREAT, EEF_STEP,
                                   JUMP_THRESHOLD);

        // Move arm
        _panda_arm_ptr->moveToPose(GOAL);

        // Open gripper
        _panda_gripper_ptr->move(PandaGripper::OPEN);

        // Move to post-place-retrait pose
        _panda_arm_ptr->linearMove(POST_PLACE_RETREAT, EEF_STEP,
                                   JUMP_THRESHOLD);

    } catch (PandaArmErr &e) {
        throw PandaErr("place()", e.what());

    } catch (PandaGripperErr &e) {
        throw PandaErr("place()", e.what());
    }
}

}  // namespace panda_controller