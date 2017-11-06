#ifndef BUILDINFO_H
#define BUILDINFO_H
#include <string>
namespace  ros2_components {

/**
 * @brief The BuildInfo class provides a string that contains the date and time of the build. (The time when this file was processed by the preprocessor)
 */
class BuildInfo
{
public:
    static std::string get_build_date()
    {
        return std::string(__DATE__) + "  " + std::string(__TIME__);
    }
};
}
#endif // BUILDINFO_H
