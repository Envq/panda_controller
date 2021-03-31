// panda_controller
#include "panda_controller/colors.hpp"



// CLASSES ====================================================================
std::string Colors::getColorFG(const int &R, const int &G, const int &B) {
    std::string res = "\033[38;2;";
    res += std::to_string(R) + ";";
    res += std::to_string(G) + ";";
    res += std::to_string(B) + "m";
    return res;
}


std::string Colors::getColorBG(const int &R, const int &G, const int &B) {
    std::string res = "\033[48;2;";
    res += std::to_string(R) + ";";
    res += std::to_string(G) + ";";
    res += std::to_string(B) + "m";
    return res;
}


void Colors::printColorsTest() {
    for (int i = 30; i < 37; i++) {
        std::cout << static_cast<Colors::Code>(i) << "FOREGROUND NORMAL"
                  << Colors::Code::RESET << std::endl;
    }
    for (int i = 40; i < 47; i++) {
        std::cout << static_cast<Colors::Code>(i) << "BACKGROUND NORMAL"
                  << Colors::Code::RESET << std::endl;
    }
    for (int i = 90; i < 97; i++) {
        std::cout << static_cast<Colors::Code>(i) << "FOREGROUND BRIGHT"
                  << Colors::Code::RESET << std::endl;
    }
    for (int i = 100; i < 107; i++) {
        std::cout << static_cast<Colors::Code>(i) << "BACKGROUND BRIGHT"
                  << Colors::Code::RESET << std::endl;
    }
}


std::ostream &operator<<(std::ostream &stream, const Colors::Code &color) {
    stream << "\033[" << static_cast<int>(color) << "m";
    return stream;
}



// FUNCTIONS ==================================================================
std::string colorize(const std::string &MSG, const Colors::Code &COLOR) {
    return "\033[1;" + std::to_string(static_cast<int>(COLOR)) + 'm' + MSG +
           "\033[0m";
}


void print_col(const std::string MSG, const Colors::Code &COLOR) {
    std::cout << colorize(MSG, COLOR) << std::endl;
}