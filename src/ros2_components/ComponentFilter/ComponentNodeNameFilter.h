#ifndef COMPONENTNAMEFILTER_H
#define COMPONENTNAMEFILTER_H

#include "ComponentListFilter.h"
namespace ros2_components {


class ComponentNodeNameFilter : public ComponentListFilter
{
public:
    ComponentNodeNameFilter(std::string _NodeName);
    std::string getNodeName() const;
    void setNodeName(const std::string &value);

     virtual std::vector<ComponentInfo> Filter(std::vector<ComponentInfo> infos);

private:
    std::string NodeName;
};
}
#endif // COMPONENTNAMEFILTER_H
