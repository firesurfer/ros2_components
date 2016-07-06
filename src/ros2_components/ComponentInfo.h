#ifndef COMPONENTINFO_H
#define COMPONENTINFO_H

#include <QObject>
namespace ros2_components
{
struct ComponentInfo
{

public:
    ComponentInfo();
    ComponentInfo(const ComponentInfo &info);
    uint64_t id;
    std::string type;
    std::string name;
    int64_t parentId;
    std::string parentType;
    std::vector<int64_t> childIds;
    std::vector<std::string> childTypes;
    int64_t machineip;
    std::string nodename;

};

}

#endif // COMPONENTINFO_H
