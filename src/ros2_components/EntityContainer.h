/*
 * Copyright 2018 <Lennart Nachtigall> <firesurfer127@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#pragma once

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


