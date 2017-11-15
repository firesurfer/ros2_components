#ifndef ENTITYCONTAINER_H
#define ENTITYCONTAINER_H

#include "EntityBase.h"
#include "EntityCastException.h"
namespace ros2_components
{
/**
 * @brief The EntityContainer class
 * This is a helper class that wraps a shared pointer to an Entity.
 * It provides helper functions for determine the type and casting the Entity.
 */
class EntityContainer
{
public:
    EntityContainer(EntityBase::SharedPtr _entity)
    {
        this->instance =_entity;
        this->className = _entity->getClassName();
    }

    template<class U>
    bool isType()
    {
        return dynamic_pointer_cast<U>(instance) != nullptr;
    }

    /**
     * @brief Cast internal stored entity to given type
     * @throws EntityCastException if the cast was unsucessfull
     */
    template<class U>
    std::shared_ptr<U> cast()
    {
        std::shared_ptr<U> ent = dynamic_pointer_cast<U>(instance);
        if(!ent)
            throw EntityCastException();
        return ent;
    }
    /**
     * @brief getClassName
     * @return The name of the class
     */
    std::string getClassName() const;
    /**
     * @brief getEntityName
     * @return The name of the Entity
     */
    std::string getEntityName() const;

private:
    EntityBase::SharedPtr instance;
    std::string className;
};
}

#endif // ENTITYCONTAINER_H
