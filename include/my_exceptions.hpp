#pragma once

// C++
#include <stdexcept>



//#############################################################################
namespace my_exceptions {


// Execeptions for data_manager
class data_manager_error : public std::runtime_error {
  public:
    data_manager_error(const std::string &msg = "Poses manager error")
        : std::runtime_error(msg) {}
};


// Execeptions for arm
class arm_error : public std::runtime_error {
  public:
    arm_error(const std::string &msg = "Arm error") : std::runtime_error(msg) {}
};


}  // namespace my_exceptions


class collision_object_creation_error : public std::runtime_error {
  public:
    collision_object_creation_error(
        const std::string &msg = "Collision Object creation error")
        : std::runtime_error(msg) {}
};


class json_field_error : public std::runtime_error {
  public:
    json_field_error(const std::string &msg = "Json field error")
        : std::runtime_error(msg) {}
};


class planning_error : public std::runtime_error {
  public:
    planning_error(const std::string &msg = "Planning error")
        : std::runtime_error(msg) {}
};