#include "ComponentNameFilter.h"
namespace ros2_components
{
ComponentNameFilter::ComponentNameFilter(std::__cxx11::string _Name)
{

}

std::string ComponentNameFilter::getName() const
{
    return Name;
}

void ComponentNameFilter::setName(const std::string &value)
{
    Name = value;
}

std::vector<ComponentInfo> ComponentNameFilter::Filter(std::vector<ComponentInfo> infos)
{
    std::vector<ComponentInfo> components;
    for(ComponentInfo & info: infos)
    {
        if(info.name == this->Name)
            components.push_back(info);
    }
    return components;
}

}
