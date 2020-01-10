#pragma once

// C++
#include <stdexcept>



//#############################################################################
namespace my_exceptions {



//#############################################################################
// CONSTANTS
const std::string DIVISOR = "\n --> ";



//#############################################################################
// EXCEPTIONS

// Execeptions for data_manager
class data_manager_error : public std::runtime_error {
  public:
    data_manager_error(const std::string &MSG)
        : std::runtime_error("data_manager_error()" + DIVISOR + MSG) {
    }
};


// Execeptions for Panda
class panda_error : public std::runtime_error {
  public:
    panda_error(const std::string &MSG)
        : std::runtime_error("panda_error()" + DIVISOR + MSG) {
    }
};


// Execeptions for Panda arm
class panda_arm_error : public panda_error {
  public:
    panda_arm_error(const std::string &MSG)
        : panda_error("panda_arm_error()" + DIVISOR + MSG) {
    }
};


// Execeptions for Panda gripper
class panda_gripper_error : public panda_error {
  public:
    panda_gripper_error(const std::string &MSG)
        : panda_error("panda_gripper_error()" + DIVISOR + MSG) {
    }
};


//#############################################################################
// FUNCTIONS
inline std::string get_err_msg(const std::string &NAME,
                               const std::string &MSG) {
    return "In: " + NAME + ".cpp" + DIVISOR + MSG;
}

}  // namespace my_exceptions