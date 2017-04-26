#ifndef ENTITYCONTAINER_H
#define ENTITYCONTAINER_H

#include "EntityBase.h"

namespace ros2_components
{
class EntityContainer
{
public:
    EntityContainer(EntityBase::SharedPtr _entity)
    {
        this->Instance =_entity;
        this->ClassName = _entity->getClassName();

    }

    template<class U>
    bool IsType()
    {
        return dynamic_pointer_cast<U>(Instance) != nullptr;
    }

    template<class U>
    std::shared_ptr<U> Cast()
    {
        return dynamic_pointer_cast<U>(Instance);
    }

    std::string getClassName() const
    {
         return ClassName;
    }

private:
    EntityBase::SharedPtr Instance;
    std::string ClassName;
};
}

#endif // ENTITYCONTAINER_H
