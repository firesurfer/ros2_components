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

