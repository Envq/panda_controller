/**
 * @file colors.hpp
 * @author Enrico Sgarbanti
 * @brief This file contains the classes and function that allows you to manage
 * the font colors on the terminal.
 * @version 0.1
 * @date 20-02-2020
 *
 * @copyright Copyright (c) 2020 by Enrico Sgarbanti. License GPLv3.
 *
 */
#pragma once

// C++
#include <sstream>
#include <string>

// ROS
#include <ros/ros.h>



//#############################################################################
// CONFIGS ####################################################################
/// @brief This namespace contains the configurations of this file.
namespace config {
const std::string START_STRING = "## ";
}  // namespace config



//#############################################################################
// ENUMERATIONS ###############################################################
/// @brief This class contains the string to be used to modify the terminal.
/// output.
class Colors {
  public:
    enum Code {
        FG_DEFAULT = 39,
        BG_DEFAULT = 49,
        FG_BLACK = 30,
        BG_BLACK = 40,
        FG_RED = 31,
        BG_RED = 41,
        FG_GREEN = 32,
        BG_GREEN = 42,
        FG_YELLOW = 33,
        BG_YELLOW = 43,
        FG_BLUE = 34,
        BG_BLUE = 44,
        FG_MAGENTA = 35,
        BG_MAGENTA = 45,
        FG_CYAN = 36,
        BG_CYAN = 46,
        FG_WHITE = 37,
        BG_WHITE = 47,
        FG_BLACK_BRIGHT = 90,
        BG_BLACK_BRIGHT = 100,
        FG_RED_BRIGHT = 91,
        BG_RED_BRIGHT = 101,
        FG_GREEN_BRIGHT = 92,
        BG_GREEN_BRIGHT = 102,
        FG_YELLOW_BRIGHT = 93,
        BG_YELLOW_BRIGHT = 103,
        FG_BLUE_BRIGHT = 94,
        BG_BLUE_BRIGHT = 104,
        FG_MAGENTA_BRIGHT = 95,
        BG_MAGENTA_BRIGHT = 105,
        FG_CYAN_BRIGHT = 96,
        BG_CYAN_BRIGHT = 106,
        FG_WHITE_BRIGHT = 97,
        BG_WHITE_BRIGHT = 107,
        RESET = 0,
        BOLD_INTENSITY = 1,
        FAINT_INTENSITY = 2,
        INTENSITY_OFF = 22,
        FAINT_OFF = 1,
        ITALICS = 3,  // not widely supported
        ITALICS_OFF = 23,
        UNDERLINE = 4,
        DOUBLE_UNDERLINE = 21,
        UNDERLINE_OFF = 24,
        BLINK_SLOW = 5,
        BLINK_RAPID = 6,  // not widely supported
        BLINK_OFF = 25,
        INVERSE = 7,
        INVERSE_OFF = 27,
        CROSSEDOUT = 9,
        CROSSEDOUT_OFF = 29
    };

    /**
     * @brief Construct a new Colors object.
     *
     */
    Colors() = default;

    /**
     * @brief Returns the string that changes the foreground color of the
     * specified RGB color.
     *
     * @param R red value.
     * @param G green value.
     * @param B blue value.
     * @return std::string string to change the foreground color.
     */
    static std::string getColorFG(const int &R, const int &G, const int &B);

    /**
     * @brief Returns the string that changes the background color of the
     * specified RGB color.
     *
     * @param R red value.
     * @param G green value.
     * @param B blue value.
     * @return std::string string to change the background color.
     */
    static std::string getColorBG(const int &R, const int &G, const int &B);

    /**
     * @brief This method print a test of colors.
     *
     */
    static void printColorsTest();

    /**
     * @brief Operator overload << for easier modification of the terminal
     * output. Example to change the foreground color to red: std::cout <<
     * Colors::FG_RED << "text" << Colors::RESET << std::endl;
     *
     * @param stream
     * @param color
     * @return std::ostream&
     */
    friend std::ostream &operator<<(std::ostream &stream,
                                    const Colors::Code &color);
};


//#############################################################################
// FUNCTIONS ##################################################################
/**
 * @brief Function to extend ROS_INFO_STREAM with a START_STRING (e.g.: ##)
 * and with foreground and background color.
 *
 * @tparam Args
 * @param COLOR_FG foreground color.
 * @param COLOR_BG background color.
 * @param args strings to be printed.
 */
template<typename... Args>
void ROS_STRONG_INFO(const Colors::Code &COLOR_FG, const Colors::Code &COLOR_BG,
                     Args &&... args) {
    std::stringstream ss;
    ss << Colors::BOLD_INTENSITY;
    ss << COLOR_FG;
    ss << COLOR_BG;
    ss << config::START_STRING;

    using expander = int[];
    (void)expander{0, (void(ss << std::forward<Args>(args)), 0)...};

    ss << Colors::RESET;
    ROS_INFO_STREAM(ss.str());
}