#include "arm.hpp"



//#############################################################################
// PRIVATE FUNCTIONS AND ENUMERATIONS

trajectory_msgs::JointTrajectory
get_gripper_trajectory(const float &WIDTH, const float &SPEED = 1.0);



//#############################################################################
// PUBLIC FUNCTIONS IMPLEMENTATIONS
namespace arm {
geometry_msgs::Vector3 get_vector_with(const std::string &type) {
    geometry_msgs::Vector3 vector;

    if (type == "pos_x") {
        vector.x = 1.0;

    } else if (type == "neg_x") {
        vector.x = -1.0;

    } else if (type == "pos_y") {
        vector.y = 1.0;

    } else if (type == "neg_y") {
        vector.y = -1.0;

    } else if (type == "pos_z") {
        vector.z = 1.0;

    } else if (type == "neg_z") {
        vector.z = -1.0;
    }

    return std::move(vector);
}

}  // namespace arm



//#############################################################################
// PUBLIC METHODS IMPLEMENTATIONS
namespace arm {

Panda::Panda(const std::string &PANDA_GROUP) {
    // Init MoveGroupInterface
    try {
        move_group_ptr.reset(
            new moveit::planning_interface::MoveGroupInterface(PANDA_GROUP));

    } catch (const std::runtime_error &e) {
        throw my_exceptions::arm_error(
            "Impossible initialize MoveGroupInterface");
    }

    // Init PlanningSceneInterface
    try {
        planning_scene_ptr.reset(
            new moveit::planning_interface::PlanningSceneInterface);

    } catch (const std::runtime_error &e) {
        throw my_exceptions::arm_error(
            "Impossible initialize PlanningSceneInterface");
    }
}


geometry_msgs::Pose Panda::getCurrentPose() {
    return move_group_ptr->getCurrentPose().pose;
}


void Panda::setSpeed(const float &SPEED) {
    move_group_ptr->setMaxVelocityScalingFactor(SPEED);
    speed = SPEED;
}


void Panda::setScene(const moveit_msgs::PlanningScene &SCENE) {
    planning_scene_ptr->applyPlanningScene(SCENE);
}


void Panda::resetScene() {
    moveit_msgs::PlanningScene empy_scene;
    planning_scene_ptr->applyPlanningScene(empy_scene);
}

void Panda::moveToPosition(const geometry_msgs::Pose &POSE,
                           const bool &PLAN_ONLY) {
    // Set the target Pose
    move_group_ptr->setPoseTarget(POSE);

    // Perform the planning
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto res = move_group_ptr->plan(plan);

    // Check the state of plainning
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw my_exceptions::arm_error("Planning failure");

    // Execute the move
    if (!PLAN_ONLY)
        move_group_ptr->move();
}


void Panda::pick(const std::string &OBJECT_NAME,
                 const std::string &PICK_SURFACE,
                 const geometry_msgs::Vector3 &pre_vector,
                 const geometry_msgs::Vector3 &post_vector,
                 const geometry_msgs::Pose &POSE, const bool &PLAN_ONLY) {
    // Init grasp pose
    geometry_msgs::PoseStamped grasp_pose;
    grasp_pose.header.frame_id = arm::FRAME_REF;
    grasp_pose.pose = POSE;

    // Init pre-grasp approach
    moveit_msgs::GripperTranslation pre;
    pre.direction.header.frame_id = arm::FRAME_REF;
    pre.direction.vector = pre_vector;
    pre.min_distance = 0.095;
    pre.desired_distance = 0.115;

    // Init post-grasp retreat
    moveit_msgs::GripperTranslation post;
    post.direction.header.frame_id = arm::FRAME_REF;
    post.direction.vector = post_vector;
    post.min_distance = 0.1;
    post.desired_distance = 0.25;

    // Init grasp
    moveit_msgs::Grasp grasp;
    grasp.grasp_pose = grasp_pose;
    grasp.pre_grasp_approach = pre;
    grasp.post_grasp_retreat = post;
    grasp.pre_grasp_posture = get_gripper_trajectory(
        GRIPPER_MAX_WIDTH, Panda::speed);  // Open gripper
    grasp.grasp_posture = get_gripper_trajectory(
        0.01 - arm::DELTA_GRASP, Panda::speed);  // Close gripper


    // Set support surface to prevent collision with surface
    move_group_ptr->setSupportSurfaceName(PICK_SURFACE);

    // Call pick to pick up the object using the grasps given
    move_group_ptr->pick(OBJECT_NAME, grasp, PLAN_ONLY);
}


void Panda::place(const std::string &OBJECT_NAME,
                  const std::string &PLACE_SURFACE,
                  const geometry_msgs::Vector3 &PRE_VECTOR,
                  const geometry_msgs::Vector3 &POST_VECTOR,
                  const geometry_msgs::Pose &POSE, const bool &PLAN_ONLY) {
    // Init place pose
    geometry_msgs::PoseStamped place_pose;
    place_pose.header.frame_id = arm::FRAME_REF;
    place_pose.pose = POSE;

    // Init pre-place approach
    moveit_msgs::GripperTranslation pre;
    pre.direction.header.frame_id = arm::FRAME_REF;
    pre.direction.vector = PRE_VECTOR;
    pre.min_distance = 0.095;
    pre.desired_distance = 0.115;

    // Init post-place retreat
    moveit_msgs::GripperTranslation post;
    post.direction.header.frame_id = arm::FRAME_REF;
    post.direction.vector = POST_VECTOR;
    post.min_distance = 0.1;
    post.desired_distance = 0.25;

    // Init place
    moveit_msgs::PlaceLocation place;
    place.place_pose = place_pose;
    place.pre_place_approach = pre;
    place.post_place_retreat = post;
    place.post_place_posture = get_gripper_trajectory(
        GRIPPER_MAX_WIDTH, Panda::speed);  // Open gripper
    
    // Init places vector
    std::vector<moveit_msgs::PlaceLocation> places;
    places.push_back(place);

    // Set support surface to prevent collision with surface
    move_group_ptr->setSupportSurfaceName(PLACE_SURFACE);

    // Call place to place the object using the place locations given
    move_group_ptr->place(OBJECT_NAME, places, PLAN_ONLY);
}

}  // namespace arm



//#############################################################################
// PRIVATE FUNCTIONS IMPLEMENTATIONS
trajectory_msgs::JointTrajectory get_gripper_trajectory(const float &WIDTH,
                                                        const float &SPEED) {
    // Create trajectory
    trajectory_msgs::JointTrajectory posture;

    // Insert the finger joints of panda robot
    posture.joint_names.push_back(arm::FINGER_SX);
    posture.joint_names.push_back(arm::FINGER_DX);

    // Create point of trajectory
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(0.5);
    // -- Add info for FINGER1
    point.positions.push_back(WIDTH);
    point.velocities.push_back(SPEED);
    // -- Add info for FINGER2
    point.positions.push_back(WIDTH);
    point.velocities.push_back(SPEED);

    // Add trajectory point on posture
    posture.points.push_back(point);

    // TODO: there is MIMIC in urdf!

    return std::move(posture);
}