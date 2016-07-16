#ifndef COMPONENTINFO_H
#define COMPONENTINFO_H

#include <QObject>
/**
 * Represents information about a certain component (e.g a motor)
 * These information allow to  definitely identify any component/entity in the system
 *
 * TODO Move the name from ComponentInfo to EntityInfo
 */
namespace ros2_components
{
struct ComponentInfo
{

public:
    ComponentInfo();
    //Copy constructor
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
