#pragma once

// C++
#include <stdexcept>



//#############################################################################
// NAMESPACE ##################################################################
namespace PCEXC {



//#############################################################################
// CONSTANTS ##################################################################
const std::string DIVISOR = "\n --> ";



//#############################################################################
// CLASSES ####################################################################

// Exceptions for data_manager
class data_manager_error : public std::runtime_error {
  public:
    data_manager_error(const std::string &MSG)
        : std::runtime_error("data_manager_error()" + DIVISOR + MSG) {
    }
};


// Exceptions for Panda
class panda_error : public std::runtime_error {
  public:
    panda_error(const std::string &MSG)
        : std::runtime_error("panda_error()" + DIVISOR + MSG) {
    }
};


// Exceptions for Panda arm
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
// INLINE FUNCTIONS ###########################################################
inline std::string get_err_msg(const std::string &NAME,
                               const std::string &MSG) {
    return "In: " + NAME + ".cpp" + DIVISOR + MSG;
}


}  // namespace PCEXC