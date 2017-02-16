#include "ComponentListFilter.h"
namespace ros2_components
{
ComponentListFilter::ComponentListFilter()
{

}

std::vector<ComponentInfo> ComponentListFilter::Filter(std::vector<ros2_components::ComponentInfo> infos)
{
    throw std::runtime_error("You have to inherit from ComponentListFilter in order to implement a specific filter!");
}
}
