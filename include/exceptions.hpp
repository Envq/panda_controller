#pragma once

// C++
#include <sstream>
#include <stdexcept>



//#############################################################################
// NAMESPACE ##################################################################
namespace PCEXC {



//#############################################################################
// CONFIG #####################################################################
const std::string DIVISOR = "\n --> ";



//#############################################################################
// CLASSES ####################################################################

// Exceptions for data_manager
class DataManagerException : public std::exception {
  private:
    std::string msg_;

  public:
    template<typename... Args> DataManagerException(Args &&... args) {
        std::stringstream ss;
        ss << "DataManagerException()";
        using expander = int[];
        (void)expander{0,
                       (void(ss << DIVISOR << std::forward<Args>(args)), 0)...};
        msg_ = ss.str();
    }

    const char *what() const throw() {
        return msg_.c_str();
    }
};

// Exceptions for Panda arm
class PandaArmException : public std::exception {
  private:
    std::string msg_;

  public:
    template<typename... Args> PandaArmException(Args &&... args) {
        std::stringstream ss;
        ss << "PandaArmException()";
        using expander = int[];
        (void)expander{0,
                       (void(ss << DIVISOR << std::forward<Args>(args)), 0)...};
        msg_ = ss.str();
    }

    const char *what() const throw() {
        return msg_.c_str();
    }
};

// Execeptions for Panda gripper
class PandaGripperException : public std::exception {
  private:
    std::string msg_;

  public:
    template<typename... Args> PandaGripperException(Args &&... args) {
        std::stringstream ss;
        ss << "PandaGripperException()";
        using expander = int[];
        (void)expander{0,
                       (void(ss << DIVISOR << std::forward<Args>(args)), 0)...};
        msg_ = ss.str();
    }

    const char *what() const throw() {
        return msg_.c_str();
    }
};



//#############################################################################
// INLINE FUNCTIONS ###########################################################
inline std::string get_err_msg(const std::string &NAME,
                               const std::string &MSG) {
    return "In '" + NAME + ".cpp'" + DIVISOR + MSG;
}

}  // namespace PCEXC
