#include "ComponentNodeNameFilter.h"
namespace ros2_components
{
ComponentNodeNameFilter::ComponentNodeNameFilter(std::__cxx11::string _NodeName)
{
    this->NodeName = _NodeName;
}

std::string ComponentNodeNameFilter::getNodeName() const
{
    return NodeName;
}

void ComponentNodeNameFilter::setNodeName(const std::string &value)
{
    NodeName = value;
}

std::vector<ComponentInfo> ComponentNodeNameFilter::Filter(std::vector<ComponentInfo> infos)
{
    std::vector<ComponentInfo> components;
    for(ComponentInfo & info: infos)
    {
        if(info.nodename == this->NodeName)
            components.push_back(info);
    }
    return components;
}

}
