#include "ComponentInfo.h"
namespace ros2_components
{
ComponentInfo::ComponentInfo()
{

}

ComponentInfo::ComponentInfo(const ComponentInfo&  info)
{
    this->id = info.id;
    this->childIds = info.childIds;
    this->childTypes = info.childTypes;
    this->machineip = info.machineip;
    this->name = info.name;
    this->nodename = info.nodename;
    this->parentId = info.parentId;
    this->parentType = info.parentType;
    this->type = info.type;
}

}
