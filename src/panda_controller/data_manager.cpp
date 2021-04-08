// Custom
#include "panda_controller/data_manager.hpp"



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// PRIVATE FUNCTIONS ==========================================================
std::string _get_path(const std::string &RELATIVE_PATH) {
    // Get package path
    auto PACKAGE_PATH = ros::package::getPath(data_manager::PACKAGE_NAME);

    if (PACKAGE_PATH.empty())
        throw DataManagerErr("get_path()", "Package path not found");

    // Return absolute path
    return PACKAGE_PATH + RELATIVE_PATH;
}


scene_object _create_scene_object(const YAML::Node &OBJECT) {
    // Create scene object
    scene_object scene_obj;

    // Define the color object
    scene_obj.color.id = OBJECT["name"].as<std::string>();
    scene_obj.color.color.r = OBJECT["color"]["r"].as<float>();
    scene_obj.color.color.g = OBJECT["color"]["g"].as<float>();
    scene_obj.color.color.b = OBJECT["color"]["b"].as<float>();
    scene_obj.color.color.a = OBJECT["color"]["a"].as<float>();

    // Define the pose (specified relative to world)
    geometry_msgs::Pose pose;
    pose.position.x = OBJECT["position"]["x"].as<double>();
    pose.position.y = OBJECT["position"]["y"].as<double>();
    pose.position.z = OBJECT["position"]["z"].as<double>();
    pose.orientation.x = OBJECT["orientation"]["x"].as<double>();
    pose.orientation.y = OBJECT["orientation"]["y"].as<double>();
    pose.orientation.z = OBJECT["orientation"]["z"].as<double>();
    pose.orientation.w = OBJECT["orientation"]["w"].as<double>();

    // Define collision object
    scene_obj.object.id = OBJECT["name"].as<std::string>();
    scene_obj.object.header.frame_id = "world";
    scene_obj.object.operation = scene_obj.object.ADD;

    // Get the type of object
    const auto &TYPE = OBJECT["type"].as<std::string>();

    if (TYPE == "mesh") {
        const auto &FILE_NAME = OBJECT["file"].as<std::string>();

        // Vector to scale 3D file units
        Eigen::Vector3d scale;
        scale(0) = OBJECT["scale"]["x"].as<double>();
        scale(1) = OBJECT["scale"]["y"].as<double>();
        scale(2) = OBJECT["scale"]["z"].as<double>();

        // Get mesh
        shape_msgs::Mesh mesh;
        shapes::ShapeMsg mesh_msg;
        shapes::Mesh *mesh_shape;

        mesh_shape = shapes::createMeshFromResource(
            "package://" + data_manager::PACKAGE_NAME +
                data_manager::MESHES_FOLDER_REL_PATH + FILE_NAME,
            scale);

        // Check errors
        if (!mesh_shape)
            throw DataManagerErr("create_scene_object()",
                                 "Can't open file: " + FILE_NAME);

        shapes::constructMsgFromShape(mesh_shape, mesh_msg);
        mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

        // Push information in collision object
        scene_obj.object.meshes.push_back(mesh);
        scene_obj.object.mesh_poses.push_back(pose);

    } else {
        // Define object in the world
        shape_msgs::SolidPrimitive primitive;
        if (TYPE == "box") {
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] =
                OBJECT["dimensions"]["x"].as<double>();
            primitive.dimensions[primitive.BOX_Y] =
                OBJECT["dimensions"]["y"].as<double>();
            primitive.dimensions[primitive.BOX_Z] =
                OBJECT["dimensions"]["z"].as<double>();

        } else if (TYPE == "sphere") {
            primitive.type = primitive.SPHERE;
            primitive.dimensions.resize(1);
            primitive.dimensions[primitive.SPHERE_RADIUS] =
                OBJECT["dimensions"]["r"].as<double>();

        } else if (TYPE == "cylinder") {
            primitive.type = primitive.CYLINDER;
            primitive.dimensions.resize(2);
            primitive.dimensions[primitive.CYLINDER_HEIGHT] =
                OBJECT["dimensions"]["h"].as<double>();
            primitive.dimensions[primitive.CYLINDER_RADIUS] =
                OBJECT["dimensions"]["r"].as<double>();

        } else if (TYPE == "cone") {
            primitive.type = primitive.CONE;
            primitive.dimensions.resize(2);
            primitive.dimensions[primitive.CONE_HEIGHT] =
                OBJECT["dimensions"]["h"].as<double>();
            primitive.dimensions[primitive.CONE_RADIUS] =
                OBJECT["dimensions"]["r"].as<double>();

        } else {
            throw DataManagerErr("create_scene_object()", "Invalid type selected");
        }

        // Push information in collision object
        scene_obj.object.primitives.push_back(primitive);
        scene_obj.object.primitive_poses.push_back(pose);
    }

    // Return
    return scene_obj;
}



// PUBLIC FUNCTIONS ===========================================================
geometry_msgs::Pose load_pose(const std::string &NAME) {
    // Get file path
    const std::string FILE_PATH = _get_path(data_manager::POSES_REL_PATH);

    try {
        // Load file
        auto root = YAML::LoadFile(FILE_PATH);

        // Check if pose name is correct
        if (!root[NAME])
            throw DataManagerErr("load_pose()", "'" + NAME + "' not found");

        // Get pose
        geometry_msgs::Pose pose;
        pose.position.x = root[NAME]["position"]["x"].as<double>();
        pose.position.y = root[NAME]["position"]["y"].as<double>();
        pose.position.z = root[NAME]["position"]["z"].as<double>();
        pose.orientation.x = root[NAME]["orientation"]["x"].as<double>();
        pose.orientation.y = root[NAME]["orientation"]["y"].as<double>();
        pose.orientation.z = root[NAME]["orientation"]["z"].as<double>();
        pose.orientation.w = root[NAME]["orientation"]["w"].as<double>();

        // Return the pose
        return pose;

    } catch (const YAML::BadFile &e) {
        throw DataManagerErr("load_pose()",
                             "YAML:", "Can't open the file: " + FILE_PATH);

    } catch (const YAML::InvalidNode &e) {
        throw DataManagerErr("load_pose()", "YAML:"
                                            "Badly formed file: " +
                                                FILE_PATH);

    } catch (const YAML::BadConversion &e) {
        throw DataManagerErr("load_pose()",
                             "YAML:", "Conversion failed while reading: " + NAME);
    }
}


void save_pose(const geometry_msgs::Pose &POSE, const std::string &NAME) {
    // Get file path
    const std::string FILE_PATH = _get_path(data_manager::POSES_REL_PATH);

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
            throw DataManagerErr("save_pose()", "ofstream failure:",
                                 "Can't write to the file: " + FILE_PATH);
        fout << root;
        fout.close();

    } catch (const YAML::BadFile &e) {
        throw DataManagerErr("save_pose()",
                             "YAML:", "Can't open the file: " + FILE_PATH);
    }
}


moveit_msgs::PlanningScene load_scene(const std::string &NAME) {
    // Get file path
    const std::string FILE_PATH = _get_path(data_manager::SCENES_REL_PATH);

    try {
        // Load file
        auto root = YAML::LoadFile(FILE_PATH);

        // Check if scene name is correct
        if (!root[NAME])
            throw DataManagerErr("load_scene()", "'" + NAME + "' not found");

        // Get scene
        moveit_msgs::PlanningScene scene;
        scene.name = NAME;

        // Get collision objectes
        for (const auto &object : root[NAME]) {
            // Create scene object
            auto scene_object = _create_scene_object(object);

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
        throw DataManagerErr("load_scene()",
                             "YAML:", "Can't open the file: " + FILE_PATH);

    } catch (const YAML::InvalidNode &e) {
        throw DataManagerErr("load_scene()",
                             "YAML:", "Badly formed file: " + FILE_PATH);

    } catch (const YAML::BadConversion &e) {
        throw DataManagerErr("load_scene()",
                             "YAML:", "Conversion failed while reading: " + NAME);
    }
}

}  // namespace panda_controller