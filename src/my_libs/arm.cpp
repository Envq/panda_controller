#include "arm.hpp"



//#############################################################################
// PUBLIC METHODS IMPLEMENTATIONS

namespace arm {

Panda::Panda(const std::string &panda_group) {
    try {
        move_group_ptr.reset(
            new moveit::planning_interface::MoveGroupInterface(panda_group));

    } catch (const std::runtime_error &e) {
        throw my_exceptions::arm_error("Impossible initialize moveGroupInterface");
    }
}


geometry_msgs::Pose Panda::getCurrentPose() {
    return move_group_ptr->getCurrentPose().pose;
}


void Panda::moveToPosition(const geometry_msgs::Pose &pose, const float &speed,
                           const bool &active_moving) {
    // Set the target Pose
    move_group_ptr->setPoseTarget(pose);

    // Set the speed
    move_group_ptr->setMaxVelocityScalingFactor(speed);

    // Perform the planning
    auto res = move_group_ptr->plan(plan);

    // Check the state of plainning
    if (res != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw my_exceptions::arm_error("Planning failure");

    // Execute the move
    if (active_moving)
        move_group_ptr->move();
}


void moveGripper(const double len, const float &speed,
                 const bool &active_moving) {}


void Panda::setGripper(trajectory_msgs::JointTrajectory &posture, bool open) {
    // Add both finger joints of panda robot
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    // Set them as open, wide enough for the object to fit
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = open ? 0.04 : 0.00;
    posture.points[0].positions[1] = open ? 0.04 : 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
}


void Panda::pick(const geometry_msgs::Pose &pose) {
    // Create a vector of grasps to be attempted, currently only creating
    // single grasp. This is essentially useful when using a grasp generator
    // to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);


    //** Setting grasp pose
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = pose.position.x;
    grasps[0].grasp_pose.pose.position.y = pose.position.y;
    grasps[0].grasp_pose.pose.position.z = pose.position.z;


    //** Setting pre-grasp approach
    // Defined with respect to frame_id
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    // Direction is set as positive x axis
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;


    //** Setting post-grasp retreat
    // Defined with respect to frame_id
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    // Direction is set as positive z axis
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;


    // Setting posture of eef before grasp
    setGripper(grasps[0].pre_grasp_posture, 1);
    // Setting posture of eef during grasp
    setGripper(grasps[0].grasp_posture, 0);


    // Set support surface
    // move_group_ptr->setSupportSurfaceName("table1");

    // Call pick to pick up the object using the grasps given
    move_group_ptr->pick("object", grasps);
}


void Panda::place(const geometry_msgs::Pose &pose) {
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    // While placing it is the exact location of the center of the object
    place_location[0].place_pose.pose.position.x = pose.position.x;
    place_location[0].place_pose.pose.position.y = pose.position.y;
    place_location[0].place_pose.pose.position.z = pose.position.z;


    // Setting pre-place approach
    // Defined with respect to frame_id
    place_location[0].pre_place_approach.direction.header.frame_id =
        "panda_link0";
    // Direction is set as negative z axis
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // Defined with respect to frame_id
    place_location[0].post_place_retreat.direction.header.frame_id =
        "panda_link0";
    // Direction is set as negative y axis
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;


    // Setting posture of eef after placing object
    setGripper(place_location[0].post_place_posture, 1);


    // Set support surface
    // move_group_ptr->setSupportSurfaceName("table2");

    // Call place to place the object using the place locations given
    move_group_ptr->place("object", place_location);
}

}  // namespace arm