#ifndef UTILS_H
#define UTILS_H

#include <iostream>

class ColoredTextPrinter {
public:
    // ANSI color codes for text
    enum class TextColor {
        Black = 30,
        Red,
        Green,
        Yellow,
        Blue,
        Magenta,
        Cyan,
        White
    };

    static void print(const std::string& text, TextColor color) {
        std::cout << "\033[" << static_cast<int>(color) << "m" << text << "\033[0m" << std::endl;
    }
};

#endif // UTILS_H
