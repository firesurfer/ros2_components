#include "EntityContainer.h"


std::string ros2_components::EntityContainer::getClassName() const
{
    return className;
}

std::string ros2_components::EntityContainer::getEntityName() const
{
    return instance->getName();
}
