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
        this->instance =_entity;
        this->className = _entity->getClassName();

    }

    template<class U>
    bool IsType()
    {
        return dynamic_pointer_cast<U>(instance) != nullptr;
    }

    template<class U>
    std::shared_ptr<U> Cast()
    {
        return dynamic_pointer_cast<U>(instance);
    }

    std::string getClassName() const
    {
         return className;
    }

private:
    EntityBase::SharedPtr instance;
    std::string className;
};
}

#endif // ENTITYCONTAINER_H
