/**
 * @file exceptions.hpp
 * @author Enrico Sgarbanti
 * @brief This file contains the exceptions used in this package.
 * @version 0.1
 * @date 20-02-2020
 *
 * @copyright Copyright (c) 2020 by Enrico Sgarbanti. License GPLv3.
 *
 */
#pragma once

// C++
#include <sstream>
#include <stdexcept>



//#############################################################################
// NAMESPACE ##################################################################
/// @brief Namespace of exceptions.
namespace PCEXC {



//#############################################################################
// CONFIGS ####################################################################
/// @brief This namespace contains the configurations of this file.
namespace config {
const std::string DIVISOR = "\n --> ";
}  // namespace config



//#############################################################################
// CLASSES ####################################################################
/// @brief Exceptions for data_manager.
class DataManagerException : public std::exception {
  private:
    std::string msg_;

  public:
    /**
     * @brief Construct a new DataManagerException object with variadic
     * template.
     *
     * @tparam Args
     * @param args Each argument is divided by the DIVIDER (e.g.: \n-->).
     */
    template<typename... Args> DataManagerException(Args &&... args) {
        std::stringstream ss;
        ss << "DataManagerException()";
        using expander = int[];
        (void)expander{
            0, (void(ss << config::DIVISOR << std::forward<Args>(args)), 0)...};
        msg_ = ss.str();
    }

    const char *what() const throw() {
        return msg_.c_str();
    }
};

/// @brief Exceptions for Panda arm.
class PandaArmException : public std::exception {
  private:
    std::string msg_;

  public:
    /**
     * @brief Construct a new PandaArmException object with variadic
     * template.
     *
     * @tparam Args
     * @param args Each argument is divided by the DIVIDER (e.g.: \n-->).
     */
    template<typename... Args> PandaArmException(Args &&... args) {
        std::stringstream ss;
        ss << "PandaArmException()";
        using expander = int[];
        (void)expander{
            0, (void(ss << config::DIVISOR << std::forward<Args>(args)), 0)...};
        msg_ = ss.str();
    }

    const char *what() const throw() {
        return msg_.c_str();
    }
};

/// @brief Execeptions for Panda gripper.
class PandaGripperException : public std::exception {
  private:
    std::string msg_;

  public:
    /**
     * @brief Construct a new PandaGripperException object with variadic
     * template.
     *
     * @tparam Args
     * @param args Each argument is divided by the DIVIDER (e.g.: \n-->).
     */
    template<typename... Args> PandaGripperException(Args &&... args) {
        std::stringstream ss;
        ss << "PandaGripperException()";
        using expander = int[];
        (void)expander{
            0, (void(ss << config::DIVISOR << std::forward<Args>(args)), 0)...};
        msg_ = ss.str();
    }

    const char *what() const throw() {
        return msg_.c_str();
    }
};



//#############################################################################
// INLINE FUNCTIONS ###########################################################
/**
 * @brief Create an error message for programs that use these exceptions. It
 * adds the file name to the exception error message.
 *
 * @param NAME The name of the file that throw exception with extension (.cpp).
 * @param MSG The error message of exceptions (exception.what())
 * @return std::string The complete message of error
 */
inline std::string get_err_msg(const std::string &NAME,
                               const std::string &MSG) {
    return "In '" + NAME + "'" + config::DIVISOR + MSG;
}

}  // namespace PCEXC
