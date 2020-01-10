#include "data_manager.hpp"



//#############################################################################
// PRIVATE FUNCTIONS and STRUCTURES
struct scene_object {
    moveit_msgs::CollisionObject collision_part;
    moveit_msgs::ObjectColor color_part;
};

std::string getPath(const std::string &RELATIVE_PATH);

scene_object create_scene_object(const Json::Value &OBJECT);



//#############################################################################
// PUBLIC FUNCTIONS IMPLEMENTATION
namespace data_manager {

void save_pose(const std::string &NAME, const geometry_msgs::Pose &POSE) {
    std::fstream file;
    Json::Value root;
    Json::StyledStreamWriter writer;

    // Get path
    const std::string PATH = getPath(data_manager::POSES_RELATIVE);

    try {
        // Init root
        file.open(PATH, std::ios::in);
        if (file.is_open()) {
            file >> root;
            file.close();
        }

        // Update root
        root[NAME]["orientation"]["x"] = POSE.orientation.x;
        root[NAME]["orientation"]["y"] = POSE.orientation.y;
        root[NAME]["orientation"]["z"] = POSE.orientation.z;
        root[NAME]["orientation"]["w"] = POSE.orientation.w;
        root[NAME]["position"]["x"] = POSE.position.x;
        root[NAME]["position"]["y"] = POSE.position.y;
        root[NAME]["position"]["z"] = POSE.position.z;

        // Save the pose
        file.open(PATH, std::ios::out);
        if (!file.is_open())
            throw my_exceptions::data_manager_error("save_pose()" +
                                                    my_exceptions::DIVISOR +
                                                    "Can't write on: " + PATH);

        writer.write(file, root);
        file.close();

    } catch (const Json::RuntimeError &e) {
        throw my_exceptions::data_manager_error(
            "save_pose()" + my_exceptions::DIVISOR + "Json ill-writted");
        file.close();
    }
}


geometry_msgs::Pose get_pose(const std::string &NAME) {
    std::ifstream file;
    Json::Value root;
    geometry_msgs::Pose pose;

    // Get path
    const std::string PATH = getPath(data_manager::POSES_RELATIVE);

    // Open file
    file.open(PATH, std::ios::in);

    // Check if the file is correctly opened
    if (!file.is_open())
        throw my_exceptions::data_manager_error(
            "get_pose()" + my_exceptions::DIVISOR + "Can't open:" + PATH);

    // Get pose
    try {
        file >> root;

        // Check if root have member NAME
        if (!root.isMember(NAME))
            throw my_exceptions::data_manager_error(
                "get_pose()" + my_exceptions::DIVISOR + "'" + NAME +
                "' pose not exist");

        // Get informations
        const auto &POSITION = root[NAME]["position"];
        const auto &ORIENTATION = root[NAME]["orientation"];

        pose.position.x = POSITION["x"].asDouble();
        pose.position.y = POSITION["y"].asDouble();
        pose.position.z = POSITION["z"].asDouble();
        pose.orientation.x = ORIENTATION["x"].asDouble();
        pose.orientation.y = ORIENTATION["y"].asDouble();
        pose.orientation.z = ORIENTATION["z"].asDouble();
        pose.orientation.w = ORIENTATION["w"].asDouble();

        file.close();

    } catch (const Json::RuntimeError &e) {
        throw my_exceptions::data_manager_error(
            "get_pose()" + my_exceptions::DIVISOR + "Json ill-writted");
        file.close();
    }

    return std::move(pose);
}


moveit_msgs::PlanningScene get_scene(const std::string &NAME) {
    std::ifstream file;
    Json::Value root;
    moveit_msgs::PlanningScene scene;

    // Get path
    const std::string PATH = getPath(data_manager::SCENES_RELATIVE);

    // Open file
    file.open(PATH, std::ios::in);

    // Check if the file is correctly opened
    if (!file.is_open())
        throw my_exceptions::data_manager_error(
            "get_scene()" + my_exceptions::DIVISOR + "Can't open:" + PATH);

    // Get Scene
    try {
        file >> root;

        // Check if root have member NAME
        if (!root.isMember(NAME)) {
            file.close();
            throw my_exceptions::data_manager_error(
                "get_scene()" + my_exceptions::DIVISOR + "'" + NAME +
                "' pose not exist");
        }

        // Get Objects
        for (const auto &object : root[NAME]) {
            // Create scene object
            auto scene_obj = create_scene_object(object);

            // Add object in the word of planning scene
            scene.world.collision_objects.push_back(scene_obj.collision_part);

            // Add object color to set
            scene.object_colors.push_back(scene_obj.color_part);
        }

        // Set that the planning scene is different
        scene.is_diff = true;

        // Close stream
        file.close();

        // Return Scene
        return std::move(scene);

    } catch (const Json::RuntimeError &e) {
        file.close();
        throw my_exceptions::data_manager_error(
            "get_scene()" + my_exceptions::DIVISOR + "Json ill-writted");

    } catch (const Json::LogicError &e) {
        file.close();
        throw my_exceptions::data_manager_error(
            "get_scene()" + my_exceptions::DIVISOR +
            "Json ill-typed: " + std::string(e.what()));
    }

    file.close();
}

}  // namespace data_manager



//#############################################################################
// PRIVATE FUNCTIONS IMPLEMENTATIONS
std::string getPath(const std::string &RELATIVE_PATH) {
    // Get package path
    auto PACKAGE_PATH = ros::package::getPath(data_manager::PACKAGE_NAME);

    if (PACKAGE_PATH.empty())
        throw my_exceptions::data_manager_error(
            "getPath()" + my_exceptions::DIVISOR + "Package path not found");

    // Return absolute path
    return PACKAGE_PATH + RELATIVE_PATH;
}


scene_object create_scene_object(const Json::Value &OBJECT) {
    // Get informations
    const auto &NAME = OBJECT["name"];
    const auto &TYPE = OBJECT["type"];
    const auto &COLOR = OBJECT["color"];
    const auto &DIMENSIONS = OBJECT["dimensions"];
    const auto &POSITION = OBJECT["position"];
    const auto &ORIENTATION = OBJECT["orientation"];

    // Check fields correctness:
    if (NAME.empty() || TYPE.empty() || COLOR.empty() || DIMENSIONS.empty() ||
        POSITION.empty() || ORIENTATION.empty()) {
        throw my_exceptions::data_manager_error("create_scene_object()" +
                                                my_exceptions::DIVISOR +
                                                "Missing some json field");
    }

    // Create scene object
    scene_object object;
    auto &color_object = object.color_part;
    auto &collision_object = object.collision_part;

    // Define color object
    color_object.id = NAME.asString();
    color_object.color.r = COLOR["r"].asFloat();
    color_object.color.g = COLOR["g"].asFloat();
    color_object.color.b = COLOR["b"].asFloat();
    color_object.color.a = COLOR["a"].asFloat();

    // Define collision object
    collision_object.header.frame_id = "panda_link0";
    collision_object.id = NAME.asString();

    // Define object in the world
    shape_msgs::SolidPrimitive primitive;
    if (TYPE == "box") {
        // Check if there is empty field
        if (DIMENSIONS["x"].empty() || DIMENSIONS["y"].empty() ||
            DIMENSIONS["z"].empty())
            throw my_exceptions::data_manager_error(
                "create_scene_object()" + my_exceptions::DIVISOR +
                "Object of type BOX have some empty "
                "field");

        // Init object
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = DIMENSIONS["x"].asDouble();
        primitive.dimensions[1] = DIMENSIONS["y"].asDouble();
        primitive.dimensions[2] = DIMENSIONS["z"].asDouble();

    } else if (TYPE == "sphere") {
        // Check if there is empty field
        if (DIMENSIONS["r"].empty())
            throw my_exceptions::data_manager_error(
                "create_scene_object()" + my_exceptions::DIVISOR +
                "Object of type SPHERE have some "
                "empty field");

        // Init object
        primitive.type = primitive.SPHERE;
        primitive.dimensions.resize(1);
        primitive.dimensions[0] = DIMENSIONS["r"].asDouble();

    } else if (TYPE == "cylinder") {
        // Check if there is empty field
        if (DIMENSIONS["h"].empty() || DIMENSIONS["r"].empty())
            throw my_exceptions::data_manager_error(
                "create_scene_object()" + my_exceptions::DIVISOR +
                "Object of type CYLINDER have some "
                "empty field");

        // Init object
        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = DIMENSIONS["h"].asDouble();
        primitive.dimensions[1] = DIMENSIONS["r"].asDouble();

    } else if (TYPE == "cone") {
        // Check if there is empty field
        if (DIMENSIONS["h"].empty() || DIMENSIONS["r"].empty())
            throw my_exceptions::data_manager_error(
                "create_scene_object()" + my_exceptions::DIVISOR +
                "Object of type CONE have some empty "
                "field");

        // Init object
        primitive.type = primitive.CONE;
        primitive.dimensions.resize(2);
        primitive.dimensions[0] = DIMENSIONS["h"].asDouble();
        primitive.dimensions[1] = DIMENSIONS["r"].asDouble();

    } else {
        throw my_exceptions::data_manager_error(
            "create_scene_object()" + my_exceptions::DIVISOR +
            "In this json file there is a not valid "
            "type specified");
    }

    // Define a pose (specified relative to frame_id)
    geometry_msgs::Pose pose;
    pose.orientation.w = ORIENTATION["w"].asDouble();
    pose.orientation.x = ORIENTATION["x"].asDouble();
    pose.orientation.y = ORIENTATION["y"].asDouble();
    pose.orientation.z = ORIENTATION["z"].asDouble();
    pose.position.x = POSITION["x"].asDouble();
    pose.position.y = POSITION["y"].asDouble();
    pose.position.z = POSITION["z"].asDouble();

    // Push information in collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    // Return
    return std::move(object);
}
