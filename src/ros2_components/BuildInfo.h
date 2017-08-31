#ifndef BUILDINFO_H
#define BUILDINFO_H
#include <string>
namespace  ros2_components {

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
