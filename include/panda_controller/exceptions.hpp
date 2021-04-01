#pragma once

// C++
#include <sstream>
#include <stdexcept>

// Custom
#include "panda_controller/colors.hpp"



// NAMESPACE ==================================================================
/// @brief Namespace of panda_errors.
namespace panda_controller {



// CONFIGS ====================================================================
/// @brief This namespace contains the configurations of this file.
namespace exceptions {
const std::string DIVISOR = "\n --> ";
}  // namespace exceptions



// CLASSES ====================================================================
/// @brief Exceptions for panda_controller
class PandaControllerErr : public std::exception {
  protected:
    /**
     * @brief Construct a new PandaControllerErr.
     *
     * @param _msg message of error.
     * @param _color color code.
     */
    std::string _msg = "PandaControllerErr()";
    Colors::Code _color;

  public:
    PandaControllerErr(const std::string &MSG = "",
                       const Colors::Code &COLOR = Colors::FG_DEFAULT)
        : std::exception(), _msg(MSG), _color(COLOR) {
    }

    const char *what() const noexcept {
        return _msg.c_str();
    }

    std::string getInfo() const noexcept {
        return colorize(_msg, _color);
    }
};

/// @brief Exceptions for data_manager.
class DataManagerErr : public PandaControllerErr {
  public:
    /**
     * @brief Construct a new DataManagerErr object with variadic
     * template.
     *
     * @tparam Args
     * @param args Each argument is divided by the DIVIDER (e.g.: \n-->).
     */
    template<typename... Args> DataManagerErr(Args &&... args) {
        std::stringstream ss;
        ss << "DataManagerErr()";
        using expander = int[];
        (void)expander{
            0, (void(ss << exceptions::DIVISOR << std::forward<Args>(args)),
                0)...};
        _msg = ss.str();
    }
};

/// @brief Exceptions for PandaArm.
class PandaArmErr : public PandaControllerErr {
  public:
    /**
     * @brief Construct a new PandaArmErr object with variadic
     * template.
     *
     * @tparam Args
     * @param args Each argument is divided by the DIVIDER (e.g.: \n-->).
     */
    template<typename... Args> PandaArmErr(Args &&... args) {
        std::stringstream ss;
        ss << "PandaArmErr()";
        using expander = int[];
        (void)expander{
            0, (void(ss << exceptions::DIVISOR << std::forward<Args>(args)),
                0)...};
        _msg = ss.str();
    }
};

/// @brief Execeptions for PandaGripper.
class PandaGripperErr : public PandaControllerErr {
  public:
    /**
     * @brief Construct a new PandaGripperErr object with variadic
     * template.
     *
     * @tparam Args
     * @param args Each argument is divided by the DIVIDER (e.g.: \n-->).
     */
    template<typename... Args> PandaGripperErr(Args &&... args) {
        std::stringstream ss;
        ss << "PandaGripperErr()";
        using expander = int[];
        (void)expander{
            0, (void(ss << exceptions::DIVISOR << std::forward<Args>(args)),
                0)...};
        _msg = ss.str();
    }
};

/// @brief Exceptions for PandaScene.
class PandaSceneErr : public PandaControllerErr {
  public:
    /**
     * @brief Construct a new PandaSceneErr object with variadic
     * template.
     *
     * @tparam Args
     * @param args Each argument is divided by the DIVIDER (e.g.: \n-->).
     */
    template<typename... Args> PandaSceneErr(Args &&... args) {
        std::stringstream ss;
        ss << "DataManagerErr()";
        using expander = int[];
        (void)expander{
            0, (void(ss << exceptions::DIVISOR << std::forward<Args>(args)),
                0)...};
        _msg = ss.str();
    }
};



// FUNCTIONS ==================================================================
/**
 * @brief Create an error message for programs that use these exceptions. It
 * adds the file name to the exception error message.
 *
 * @param FILE_NAME The name of the file that throw exception with extension
 * (.cpp).
 * @param MSG The error message of exceptions (exception.what()).
 * @return std::string The complete message of error
 */
inline std::string get_err_msg(const std::string &FILE_NAME,
                               const std::string &MSG) {
    return "In '" + FILE_NAME + "':" + exceptions::DIVISOR + MSG;
}

}  // namespace panda_controller