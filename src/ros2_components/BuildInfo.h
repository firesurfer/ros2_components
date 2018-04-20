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
#ifndef BUILD_TIME
        return std::string(__DATE__) + "  " + std::string(__TIME__);
#else
        return std::string(BUILD_TIME);
#endif
    }

    static std::string get_build_version()
    {
#ifndef BUILD_VERSION
#warning "BUILD_VERSION variable not set"
#define BUILD_VERSION "not set"
#endif
        return std::string(BUILD_VERSION);

    }
};
}
#endif // BUILDINFO_H
