// Custom
#include "panda_controller/panda_scene.hpp"



// USING NAMESPACE ============================================================
using namespace panda_controller;



// CLASSES ====================================================================
PandaScene::PandaScene() {
    // Init PlanningSceneInterface
    try {
        _scene_ptr.reset(
            new moveit::planning_interface::PlanningSceneInterface);

    } catch (const std::runtime_error &err) {
        throw PandaSceneErr("PandaScene()",
                            "Impossible initialize PlanningSceneInterface");
    }
}


void PandaScene::setScene(const moveit_msgs::PlanningScene &SCENE) {
    _scene_ptr->applyPlanningScene(SCENE);
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