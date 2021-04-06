#pragma once

// Moveit
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>

// Custom
#include "panda_controller/exceptions.hpp"
#include "panda_controller/panda_arm.hpp"
#include "panda_controller/panda_gripper.hpp"
#include "utils/colors.hpp"

// BOOST
#include <boost/shared_ptr.hpp>



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CLASSES ====================================================================
/// @brief The Panda robot arm management class.
class Panda {
  private:
    std::shared_ptr<PandaArm> _arm_ptr;
    std::shared_ptr<PandaGripper> _gripper_ptr;
    moveit::planning_interface::PlanningSceneInterfacePtr _scene_ptr;

  public:
    /**
     * @brief Construct a new Panda object
     *
     * @param REAL_ROBOT If true, it uses franka_gripper.
     * @param DELAY the seconds of delay for load the TF2
     */
    explicit Panda(const bool REAL_ROBOT, const float DELAY = 1.0);


    /**
     * @brief Get the PandaArm object
     *
     * @return std::shared_ptr<PandaArm> PandaArm
     */
    std::shared_ptr<PandaArm> getArm() const;


    /**
     * @brief Get the PandaGripper object
     *
     * @return std::shared_ptr<PandaGripper> PandaGripper
     */
    std::shared_ptr<PandaGripper> getGripper() const;


    /**
     * @brief Get the  moveit::planning_interface::PlanningSceneInterfacePtr objet
     *
     * @return moveit::planning_interface::PlanningSceneInterfacePtr
     */
    moveit::planning_interface::PlanningSceneInterfacePtr getPlanningScene() const;


    /**
     * @brief Load the scene.
     *
     * @param SCENE_NAME The scene to use.
     */
    void setScene(const std::string &SCENE_NAME);


    /**
     * @brief Load a empty scene.
     *
     */
    void resetScene();


    /**
     * @brief Pick up an object with pre-grasp-approch. Moves the arm to the
     * desired position, starting from the pre-approach-pose, and proceeds
     * moving linearly with respect to the target.
     *
     * @param PRE_GRASP_APPROCH The pose of grasp-pre-approch. It contains the
     * offset from target pose in meters.
     * @param OBJECT The pose where the robot grasp object.
     * @param POST_GRASP_RETREAT The Pose of post-grasp-retreat. It contains
     * the offset from current pose in meters.
     * @param GRASP_WIDTH The width value for gripperGrasp().
     * @param GRASP_FORCE The force value for gripperGrasp().
     * @param GRASP_EPSILON_INNER The epsilon_inner value for gripperGrasp().
     * @param GRASP_EPSILON_OUTER The epsilon_outer value for gripperGrasp().
     * @param EEF_STEP Step value for linerMove().
     * @param JUMP_THRESHOLD Jumpo threshold value for linerMove().
     */
    void pick(const geometry_msgs::Pose &PRE_GRASP_APPROCH,
              const geometry_msgs::Pose &OBJECT,
              const geometry_msgs::Pose &POST_GRASP_RETREAT,
              const double &GRASP_WIDTH, const double &GRASP_FORCE = 20.0,
              const double &GRASP_EPSILON_INNER = 0.02,
              const double &GRASP_EPSILON_OUTER = 0.02, const double &STEP = 0.01,
              const double &JUMP_THRESHOLD = 0.0);


    /**
     * @brief Place the object with post-grasp-retreat and post-place-retreat.
     * Moves the arm linearly to post-grasp-pose and then move to the desired
     * position. After that proceeds moving linearly in the post-place-pose.
     *
     * @param PRE_PLACE_APPROCH The pose of place-pre-approch. It contains the
     * offset from target pose in meters.
     * @param GOAL The Pose where place the object.
     * @param POST_PLACE_RETREAT The Pose of post-place-retreat. It contains
     * the offset from target pose in meters.
     * @param EEF_STEP Step value for cartesianMovement().
     * @param JUMP_THRESHOLD Jumpo threshold value for cartesianMovement().
     */
    void place(const geometry_msgs::Pose &PRE_PLACE_APPROCH,
               const geometry_msgs::Pose &GOAL,
               const geometry_msgs::Pose &POST_PLACE_RETREAT,
               const double &EEF_STEP = 0.01, const double &JUMP_THRESHOLD = 0.0);
};

}  // namespace panda_controller