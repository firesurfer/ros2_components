#ifndef COMPONENTINFOFACTORY_H
#define COMPONENTINFOFACTORY_H

#include "EntityBase.h"
#include "ComponentInfo.h"
#include "ros2_components_msg/msg/list_components_response.hpp"

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
    static ComponentInfo FromEntity(EntityBase::SharedPtr ent);

    /**
     * @brief FromListComponentsResponseMessage
     * @param msg
     * @return
     * Creates a ComponentInfo object from a given ListComponentsResponse msg
     */
    static ComponentInfo FromListComponentsResponseMessage(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg);


};
}
#endif // COMPONENTINFOFACTORY_H
