#pragma once

#include "ComponentListFilter.h"
namespace ros2_components {


class ComponentNameFilter : public ComponentListFilter
{
public:
    ComponentNameFilter(std::string _Name);
    std::string getName() const;
    void setName(const std::string &value);

     virtual std::vector<ComponentInfo> Filter(std::vector<ComponentInfo> infos);

private:
    std::string Name;
};
}

