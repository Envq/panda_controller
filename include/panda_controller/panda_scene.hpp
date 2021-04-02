#pragma once

// Moveit
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Custom
#include "panda_controller/data_manager.hpp"
#include "panda_controller/exceptions.hpp"



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CLASSES ====================================================================
/// @brief The Panda robot interface management class.
class PandaScene {
  private:
    moveit::planning_interface::PlanningSceneInterfacePtr _scene_ptr;


  public:
    /**
     * @brief Construct a new PandaScene object.
     *
     */
    explicit PandaScene();


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
};

}  // namespace panda_controller