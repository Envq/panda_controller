#pragma once

// C++
#include <stdexcept>



//#############################################################################
namespace my_exceptions {


// Execeptions for data_manager
class data_manager_error : public std::runtime_error {
  public:
    data_manager_error(const std::string &msg = "Poses manager error")
        : std::runtime_error(msg) {
    }
};


// Execeptions for arm
class panda_arm_error : public std::runtime_error {
  public:
    panda_arm_error(const std::string &msg = "Panda arm error")
        : std::runtime_error(msg) {
    }
};


// Execeptions for gripper
class panda_gripper_error : public std::runtime_error {
  public:
    panda_gripper_error(const std::string &msg = "Panda gripper error")
        : std::runtime_error(msg) {
    }
};



}  // namespace my_exceptions