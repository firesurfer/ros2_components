#ifndef CONSOLE_COLOR_H
#define CONSOLE_COLOR_H
#include <string>
namespace ros2_components
{
typedef enum ConsoleColor {
        FG_RED      = 31,
        FG_GREEN    = 32,
        FG_BLUE     = 34,
        FG_DEFAULT  = 39,
        BG_RED      = 41,
        BG_GREEN    = 42,
        BG_BLUE     = 44,
        BG_DEFAULT  = 49
    }ConsoleColor;
    

}
#endif
