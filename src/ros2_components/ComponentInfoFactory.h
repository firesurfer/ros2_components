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
/*ros2_components*/
#include "EntityBase.h"
#include "ComponentInfo.h"
#include "Networking.h"
/*Message*/
#include "ros2_components_msg/msg/list_components_response.hpp"
#include "ros2_components_msg/msg/component_changed.hpp"





namespace ros2_components {


class ComponentInfoFactory
{
public:
    ComponentInfoFactory();
    /**
     * @brief FromEntity
     * @param ent
     * @return ComponentInfo object
     * Creates a ComponentInfo object from a given entity
     */
    static ComponentInfo fromEntity(EntityBase::SharedPtr ent);

    /**
     * @brief FromListComponentsResponseMessage
     * @param msg
     * @return
     * Creates a ComponentInfo object from a given ListComponentsResponse msg
     */
    static ComponentInfo fromListComponentsResponseMessage(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg);
    /**
     * @brief FromComponentChangedMessage
     * @param msg
     * @return
     * Creates a ComponentInfo object from a given ComponentChanged
     */
    static ComponentInfo fromComponentChangedMessage(ros2_components_msg::msg::ComponentChanged::SharedPtr msg);


};
}

