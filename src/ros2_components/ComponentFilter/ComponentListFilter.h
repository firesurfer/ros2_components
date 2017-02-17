#pragma once

#include "ComponentInfo.h"
namespace ros2_components
{
/**
 * @brief The ComponentListFilter class
 * How to use this class:
 * Inherit from this class and override the Filter function. For example filter the components by their id, node or anything else.
 */
class ComponentListFilter
{
public:
    ComponentListFilter();
    virtual std::vector<ComponentInfo> Filter(std::vector<ComponentInfo> infos);
};
}

