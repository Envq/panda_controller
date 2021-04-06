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
                 const geometry_msgs::Pose &OBJECT,
                 const geometry_msgs::Pose &POST_GRASP_RETREAT,
                 const double &GRASP_WIDTH, const double &GRASP_FORCE,
                 const double &GRASP_EPSILON_INNER,
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

        // Move to post-grasp-retrait pose
        _panda_arm_ptr->linearMove(POST_GRASP_RETREAT, EEF_STEP, JUMP_THRESHOLD);

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
        _panda_arm_ptr->moveToPose(PRE_PLACE_APPROCH);

        // Move arm
        _panda_arm_ptr->linearMove(GOAL);

        // Open gripper
        _panda_gripper_ptr->move(PandaGripper::OPEN);

        // Move to post-place-retrait pose
        _panda_arm_ptr->linearMove(POST_PLACE_RETREAT, EEF_STEP, JUMP_THRESHOLD);

    } catch (PandaArmErr &e) {
        throw PandaErr("place()", e.what());

    } catch (PandaGripperErr &e) {
        throw PandaErr("place()", e.what());
    }
}
void openGripper(trajectory_msgs::JointTrajectory &posture) {
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory &posture) {
    // BEGIN_SUB_TUTORIAL closed_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
}

void Panda::pick(const std::string &OBJECT_NAME, const double OBJECT_WIDTH,
                 const geometry_msgs::Pose &PRE_GRASP_APPROCH,
                 const geometry_msgs::Pose &GRASP,
                 const geometry_msgs::Pose &POST_GRASP_RETREAT) {
    // Initialize
    moveit_msgs::Grasp grasp_msg;

    // Grasp pose
    grasp_msg.grasp_pose.header.frame_id = "panda_link0";
    grasp_msg.grasp_pose.pose = _panda_arm_ptr->getFlangeFromTCP(GRASP);

    // Pre-grasp pose
    grasp_msg.pre_grasp_approach.direction.header.frame_id = "panda_link0";
    grasp_msg.pre_grasp_approach.direction.vector.x = 1.0;
    grasp_msg.pre_grasp_approach.direction.vector.y = 0.0;
    grasp_msg.pre_grasp_approach.direction.vector.z = 0.0;
    grasp_msg.pre_grasp_approach.min_distance = 0.095;
    grasp_msg.pre_grasp_approach.desired_distance = 0.115;

    // Post-grasp pose
    grasp_msg.post_grasp_retreat.direction.header.frame_id = "panda_link0";
    grasp_msg.post_grasp_retreat.direction.vector.x = 0.0;
    grasp_msg.post_grasp_retreat.direction.vector.y = 0.0;
    grasp_msg.post_grasp_retreat.direction.vector.z = 1.0;
    grasp_msg.post_grasp_retreat.min_distance = 0.1;
    grasp_msg.post_grasp_retreat.desired_distance = 0.25;

    // Open Gripper
    grasp_msg.pre_grasp_posture.joint_names.resize(2);
    grasp_msg.pre_grasp_posture.joint_names[0] = "panda_finger_joint1";
    grasp_msg.pre_grasp_posture.joint_names[1] = "panda_finger_joint2";
    grasp_msg.pre_grasp_posture.points.resize(1);
    grasp_msg.pre_grasp_posture.points[0].positions.resize(2);
    grasp_msg.pre_grasp_posture.points[0].positions[0] = PandaGripper::OPEN / 2.0;
    grasp_msg.pre_grasp_posture.points[0].positions[1] = PandaGripper::OPEN / 2.0;
    grasp_msg.pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);

    // Close Gripper
    grasp_msg.grasp_posture.joint_names.resize(2);
    grasp_msg.grasp_posture.joint_names[0] = "panda_finger_joint1";
    grasp_msg.grasp_posture.joint_names[1] = "panda_finger_joint2";
    grasp_msg.grasp_posture.points.resize(1);
    grasp_msg.grasp_posture.points[0].positions.resize(2);
    grasp_msg.grasp_posture.points[0].positions[0] = OBJECT_WIDTH / 2.0;
    grasp_msg.grasp_posture.points[0].positions[1] = OBJECT_WIDTH / 2.0;
    grasp_msg.grasp_posture.points[0].time_from_start = ros::Duration(0.5);

    // Set support surface
    _panda_arm_ptr->getMoveGroup()->setSupportSurfaceName("table1");

    // Performe pick task
    _panda_arm_ptr->getMoveGroup()->pick(OBJECT_NAME, grasp_msg);
}


void Panda::place(const std::string &OBJECT_NAME,
                  const geometry_msgs::Pose &PRE_PLACE_APPROCH,
                  const geometry_msgs::Pose &PLACE,
                  const geometry_msgs::Pose &POST_PLACE_RETREAT) {
    // Initialize
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Place pose
    place_location[0].place_pose.header.frame_id = "panda_link0";
    place_location[0].place_pose.pose = _panda_arm_ptr->getFlangeFromTCP(PLACE);

    // Pre-grasp pose
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    place_location[0].pre_place_approach.direction.vector.x = 0.0;
    place_location[0].pre_place_approach.direction.vector.y = 0.0;
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Post-grasp pose
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    place_location[0].post_place_retreat.direction.vector.x = 0.0;
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.direction.vector.z = 0.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // Open Gripper
    place_location[0].post_place_posture.joint_names.resize(2);
    place_location[0].post_place_posture.joint_names[0] = "panda_finger_joint1";
    place_location[0].post_place_posture.joint_names[1] = "panda_finger_joint2";
    place_location[0].post_place_posture.points.resize(1);
    place_location[0].post_place_posture.points[0].positions.resize(2);
    place_location[0].post_place_posture.points[0].positions[0] =
        PandaGripper::OPEN / 2;
    place_location[0].post_place_posture.points[0].positions[1] =
        PandaGripper::OPEN / 2;
    place_location[0].post_place_posture.points[0].time_from_start =
        ros::Duration(0.5);

    // Set support surface
    _panda_arm_ptr->getMoveGroup()->setSupportSurfaceName("table2");

    // Performe pick task
    _panda_arm_ptr->getMoveGroup()->place(OBJECT_NAME, place_location);
}

}  // namespace panda_controller
