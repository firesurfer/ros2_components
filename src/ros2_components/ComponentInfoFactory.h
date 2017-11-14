#pragma once
/*ros2_components*/
#include "EntityBase.h"
#include "ComponentInfo.h"

/*Message*/
#include "ros2_components_msg/msg/list_components_response.hpp"
#include "ros2_components_msg/msg/component_changed.hpp"

/*Qt*/
#include <QHostAddress>
#include <QHostInfo>
#include <QNetworkInterface>



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

    /**
     * @brief GetLocalIpV4
     * @return The local ipv4 address
     */
    static uint32_t getLocalIpV4();
    /**
     * @brief getLocalIpV6
     * @return  The local ipv6 address
     */
    static std::vector<uint8_t> getLocalIpV6();
};
}

