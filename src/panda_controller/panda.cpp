// Custom
#include "panda_controller/panda.hpp"



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CLASSES ====================================================================
Panda::Panda(const bool REAL_ROBOT, const float DELAY) {
    // Init PandaArm and PandaGripper
    try {
        _arm_ptr.reset(new PandaArm(DELAY));
        _gripper_ptr.reset(new PandaGripper(REAL_ROBOT));
        _scene_ptr.reset(new moveit::planning_interface::PlanningSceneInterface());
        ros::Duration(DELAY).sleep();

    } catch (const std::runtime_error &err) {
        throw PandaErr("Panda()", err.what());
    }
}


std::shared_ptr<PandaArm> Panda::getArm() const {
    return _arm_ptr;
}


std::shared_ptr<PandaGripper> Panda::getGripper() const {
    return _gripper_ptr;
}


moveit::planning_interface::PlanningSceneInterfacePtr
Panda::getPlanningScene() const {
    return _scene_ptr;
}


void Panda::setScene(const std::string &SCENE_NAME) {
    _scene_ptr->applyPlanningScene(load_scene(SCENE_NAME));
}


void Panda::resetScene() {
    // Remove attached object
    for (const auto &e : _scene_ptr->getAttachedObjects()) {
        _arm_ptr->getMoveGroup()->detachObject(e.first);
    }

    // Remove all collision objects
    _scene_ptr->removeCollisionObjects(_scene_ptr->getKnownObjectNames());
}


void Panda::pick(const geometry_msgs::Pose &PRE_GRASP_APPROCH,
                 const geometry_msgs::Pose &OBJECT,
                 const geometry_msgs::Pose &POST_GRASP_RETREAT,
                 const double &GRASP_WIDTH, const double &GRASP_FORCE,
                 const double &GRASP_EPSILON_INNER,
                 const double &GRASP_EPSILON_OUTER, const double &EEF_STEP,
                 const double &JUMP_THRESHOLD) {
    try {
        // Open gripper
        _gripper_ptr->move(PandaGripper::OPEN);

        // Move to pre-grasp-approch pose
        _arm_ptr->moveToPose(PRE_GRASP_APPROCH);

        // Move to pose
        _arm_ptr->moveToPose(OBJECT);
        // _arm_ptr->linearMove(OBJECT, EEF_STEP, JUMP_THRESHOLD);

        // Close gripper
        _gripper_ptr->grasp(GRASP_WIDTH, GRASP_FORCE, GRASP_EPSILON_INNER,
                            GRASP_EPSILON_OUTER);

        // Move to post-grasp-retrait pose
        _arm_ptr->moveToPose(POST_GRASP_RETREAT);
        // _arm_ptr->linearMove(POST_GRASP_RETREAT, EEF_STEP, JUMP_THRESHOLD);

    } catch (PandaArmErr &e) {
        throw PandaErr("pick()", e.what());

    } catch (PandaGripperErr &e) {
        throw PandaErr("pick()", e.what());
    }
}


void Panda::place(const geometry_msgs::Pose &PRE_PLACE_APPROCH,
                  const geometry_msgs::Pose &GOAL,
                  const geometry_msgs::Pose &POST_PLACE_RETREAT,
                  const double &EEF_STEP, const double &JUMP_THRESHOLD) {
    try {
        // Move to pre-place-approch pose
        _arm_ptr->moveToPose(PRE_PLACE_APPROCH);

        // Move arm
        _arm_ptr->moveToPose(GOAL);
        // _arm_ptr->linearMove(GOAL, EEF_STEP, JUMP_THRESHOLD);

        // Open gripper
        _gripper_ptr->move(PandaGripper::OPEN);

        // Move to post-place-retrait pose
        _arm_ptr->moveToPose(POST_PLACE_RETREAT);
        // _arm_ptr->linearMove(POST_PLACE_RETREAT, EEF_STEP, JUMP_THRESHOLD);

    } catch (PandaArmErr &e) {
        throw PandaErr("place()", e.what());

    } catch (PandaGripperErr &e) {
        throw PandaErr("place()", e.what());
    }
}

}  // namespace panda_controller
