// Custom
#include "panda_controller/panda_scene.hpp"


// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CLASSES ====================================================================
PandaScene::PandaScene(const float DELAY) {
    // Init PlanningSceneInterface
    try {
        _scene_ptr.reset(new moveit::planning_interface::PlanningSceneInterface);
        ros::Duration(DELAY).sleep();

    } catch (const std::runtime_error &err) {
        throw PandaSceneErr("PandaScene()",
                            "Impossible initialize PlanningSceneInterface");
    }
}


void PandaScene::setScene(const std::string &SCENE_NAME) {
    _scene_ptr->applyPlanningScene(load_scene(SCENE_NAME));
}


void PandaScene::resetScene() {
    // Remove all collision objects
    _scene_ptr->removeCollisionObjects(_scene_ptr->getKnownObjectNames());
    // Load empty scene
    moveit_msgs::PlanningScene empty_scene;
    empty_scene.is_diff = true;
    empty_scene.name = "empty";
    _scene_ptr->applyPlanningScene(empty_scene);
}

}  // namespace panda_controller