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
class panda_error : public std::runtime_error {
  public:
    panda_error(const std::string &msg = "Panda error")
        : std::runtime_error(msg) {
    }
};


}  // namespace my_exceptions