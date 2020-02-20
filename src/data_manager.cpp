/**
 * @file data_manager.cpp
 * @author Enrico Sgarbanti
 * @brief data_manager implementations
 * @version 0.1
 * @date 20-02-2020
 *
 * @copyright Copyright (c) 2020 by Enrico Sgarbanti. License GPLv3.
 *
 */
// PANDA CONTROLLER
#include "data_manager.hpp"



//#############################################################################
// GLOBAL DATA ################################################################
struct scene_object {
    moveit_msgs::CollisionObject object;
    moveit_msgs::ObjectColor color;
};



//#############################################################################
// PRIVATE FUNCTIONS IMPLEMENTATIONS ##########################################
std::string get_path(const std::string &RELATIVE_PATH) {
    // Get package path
    auto PACKAGE_PATH =
        ros::package::getPath(data_manager::config::PACKAGE_NAME);

    if (PACKAGE_PATH.empty())
        throw PCEXC::DataManagerException("get_path()",
                                          "Package path not found");

    // Return absolute path
    return PACKAGE_PATH + RELATIVE_PATH;
}


scene_object create_scene_object(const YAML::Node &OBJECT) {
    // Get informations
    const auto &NAME = OBJECT["name"].as<std::string>();
    const auto &COLOR = OBJECT["color"];
    const auto &POSITION = OBJECT["position"];
    const auto &ORIENTATION = OBJECT["orientation"];
    const auto &TYPE = OBJECT["type"].as<std::string>();

    // Create scene object
    scene_object object;
    auto &color_object = object.color;
    auto &collision_object = object.object;

    // Define the color object
    color_object.id = NAME;
    color_object.color.r = COLOR["r"].as<float>();
    color_object.color.g = COLOR["g"].as<float>();
    color_object.color.b = COLOR["b"].as<float>();
    color_object.color.a = COLOR["a"].as<float>();

    // Define the pose (specified relative to frame_id)
    geometry_msgs::Pose pose;
    pose.position.x = POSITION["x"].as<double>();
    pose.position.y = POSITION["y"].as<double>();
    pose.position.z = POSITION["z"].as<double>();
    pose.orientation.x = ORIENTATION["x"].as<double>();
    pose.orientation.y = ORIENTATION["y"].as<double>();
    pose.orientation.z = ORIENTATION["z"].as<double>();
    pose.orientation.w = ORIENTATION["w"].as<double>();

    // Define collision object
    collision_object.id = NAME;
    collision_object.header.frame_id = data_manager::config::FRAME_REF;
    collision_object.operation = collision_object.ADD;

    // Get the type of object
    if (TYPE == "mesh") {
        const auto &FILE_NAME = OBJECT["file"].as<std::string>();
        const auto &SCALE = OBJECT["scale"];

        // Vector to scale 3D file units
        Eigen::Vector3d vector_scale;
        vector_scale(0) = SCALE["x"].as<double>();
        vector_scale(1) = SCALE["y"].as<double>();
        vector_scale(2) = SCALE["z"].as<double>();

        // Get mesh
        shape_msgs::Mesh mesh;
        shapes::ShapeMsg mesh_msg;
        shapes::Mesh *mesh_shape = shapes::createMeshFromResource(
            "package://" + data_manager::config::PACKAGE_NAME +
                data_manager::config::MESHES_FOLDER_NAME + FILE_NAME,
            vector_scale);
        shapes::constructMsgFromShape(mesh_shape, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        // Push information in collision object
        collision_object.meshes.push_back(mesh);
        collision_object.mesh_poses.push_back(pose);

    } else {
        // Get informations
        const auto &DIMENSIONS = OBJECT["dimensions"];

        // Define object in the world
        shape_msgs::SolidPrimitive primitive;
        if (TYPE == "box") {
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[0] = DIMENSIONS["x"].as<double>();
            primitive.dimensions[1] = DIMENSIONS["y"].as<double>();
            primitive.dimensions[2] = DIMENSIONS["z"].as<double>();

        } else if (TYPE == "sphere") {
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[0] = DIMENSIONS["r"].as<double>();

        } else if (TYPE == "cylinder") {
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = DIMENSIONS["h"].as<double>();
            primitive.dimensions[1] = DIMENSIONS["r"].as<double>();

        } else if (TYPE == "cone") {
            primitive.type = primitive.CONE;
            primitive.dimensions.resize(2);
            primitive.dimensions[0] = DIMENSIONS["h"].as<double>();
            primitive.dimensions[1] = DIMENSIONS["r"].as<double>();

        } else {
            throw PCEXC::DataManagerException("create_scene_object()",
                                              "Invalid type selected");
        }

        // Push information in collision object
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(pose);
    }

    // Return
    return object;
}



//#############################################################################
// PUBLIC FUNCTIONS IMPLEMENTATION
// ############################################
namespace data_manager {

void save_pose(const geometry_msgs::Pose &POSE, const std::string &NAME) {
    // Get file path
    const std::string FILE_PATH = get_path(config::POSES_RELATIVE);

    try {
        // Load file
        auto root = YAML::LoadFile(FILE_PATH);

        // Update file
        root[NAME]["position"]["x"] = POSE.position.x;
        root[NAME]["position"]["y"] = POSE.position.y;
        root[NAME]["position"]["z"] = POSE.position.z;
        root[NAME]["orientation"]["x"] = POSE.orientation.x;
        root[NAME]["orientation"]["y"] = POSE.orientation.y;
        root[NAME]["orientation"]["z"] = POSE.orientation.z;
        root[NAME]["orientation"]["w"] = POSE.orientation.w;


        // Save updates
        std::ofstream fout(FILE_PATH);
        if (fout.fail())
            throw PCEXC::DataManagerException(
                "save_pose()",
                "ofstream failure:", "Can't write to the file: " + FILE_PATH);
        fout << root;
        fout.close();

    } catch (const YAML::BadFile &e) {
        throw PCEXC::DataManagerException(
            "save_pose()", "YAML:", "Can't open the file: " + FILE_PATH);
    }
}


geometry_msgs::Pose get_pose(const std::string &NAME) {
    // Get file path
    const std::string FILE_PATH = get_path(config::POSES_RELATIVE);

    try {
        // Load file
        auto root = YAML::LoadFile(FILE_PATH);

        // Get pose
        geometry_msgs::Pose pose;
        const auto &POSITION = root[NAME]["position"];
        const auto &ORIENTATION = root[NAME]["orientation"];
        pose.position.x = POSITION["x"].as<double>();
        pose.position.y = POSITION["y"].as<double>();
        pose.position.z = POSITION["z"].as<double>();
        pose.orientation.x = ORIENTATION["x"].as<double>();
        pose.orientation.y = ORIENTATION["y"].as<double>();
        pose.orientation.z = ORIENTATION["z"].as<double>();
        pose.orientation.w = ORIENTATION["w"].as<double>();

        return pose;

    } catch (const YAML::BadFile &e) {
        throw PCEXC::DataManagerException(
            "get_pose()", "YAML:", "Can't open the file: " + FILE_PATH);

    } catch (const YAML::InvalidNode &e) {
        throw PCEXC::DataManagerException("get_pose()", "YAML:"
                                                        "Badly formed file: " +
                                                            FILE_PATH);

    } catch (const YAML::BadConversion &e) {
        throw PCEXC::DataManagerException(
            "get_pose()", "YAML:", "Conversion failed while reading: " + NAME);
    }
}


moveit_msgs::PlanningScene get_scene(const std::string &NAME) {
    // Get file path
    const std::string FILE_PATH = get_path(config::SCENES_RELATIVE);

    try {
        // Load file
        auto root = YAML::LoadFile(FILE_PATH);

        // Get scene
        moveit_msgs::PlanningScene scene;
        scene.name = NAME;

        // Get collision objectes
        for (const auto &object : root[NAME]) {
            // Create scene object
            auto scene_object = create_scene_object(object);

            // Add object in the word of planning scene
            scene.world.collision_objects.push_back(scene_object.object);

            // Add object color to set
            scene.object_colors.push_back(scene_object.color);
        }

        // Set that the planning scene is different
        scene.is_diff = true;

        // Return scene
        return scene;

    } catch (const YAML::BadFile &e) {
        throw PCEXC::DataManagerException(
            "get_scene()", "YAML:", "Can't open the file: " + FILE_PATH);

    } catch (const YAML::InvalidNode &e) {
        throw PCEXC::DataManagerException(
            "get_scene()", "YAML:", "Badly formed file: " + FILE_PATH);

    } catch (const YAML::BadConversion &e) {
        throw PCEXC::DataManagerException(
            "get_scene()", "YAML:", "Conversion failed while reading: " + NAME);
    }
}

}  // namespace data_manager