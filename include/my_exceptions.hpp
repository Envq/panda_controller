#pragma once

// C++
#include <stdexcept>



//#############################################################################
class poses_manager_error : public std::runtime_error {
  public:
    poses_manager_error(
        const std::string &msg = "Poses manager error")
        : std::runtime_error(msg) {}
};