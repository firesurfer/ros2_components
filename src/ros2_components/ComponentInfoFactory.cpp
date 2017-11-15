#include "ComponentInfoFactory.h"
namespace ros2_components
{
ComponentInfoFactory::ComponentInfoFactory()
{

}

ComponentInfo ComponentInfoFactory::fromEntity(EntityBase::SharedPtr ent)
{
    ComponentInfo info;
    info.name = ent->getName();
    info.id = ent->getId();

    //TODO move this into a more general place

    info.machineip = Networking::getLocalIpV4();
    if(ent->getParent() != NULL)
    {
        info.parentId = ent->getParent()->getId();
        info.parentType = ent->getParent()->getClassName();
    }
    else
    {
        info.parentId = -1;
        info.parentType = "";
    }
    info.type = ent->getClassName();

    for(EntityBase::SharedPtr & child : ent->getAllChilds())
    {
        info.childIds.push_back(child->getId());
        info.childTypes.push_back(child->getClassName());
    }
    info.name = ent->getName();
    info.subscriber = ent->isSubscriber();

    return info;

}

ComponentInfo ComponentInfoFactory::fromListComponentsResponseMessage(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg)
{
    ComponentInfo info;
    info.id = msg->id;
    info.name = msg->componentname;
    info.parentId = msg->parent;
    info.parentType = msg->parenttype;
    info.childIds = msg->childids;
    info.childTypes = msg->childtypes;
    info.nodename = msg->nodename;
    info.type = msg->type;
    info.machineip = msg->machineip;
    info.subscriber = msg->subscriber;
    //TODO check if all fields are used
    return info;
}

ComponentInfo ComponentInfoFactory::fromComponentChangedMessage(ros2_components_msg::msg::ComponentChanged::SharedPtr msg)
{
    ComponentInfo info;
    info.id = msg->id;
    info.name = msg->componentname;
    info.parentId = msg->parent;
    info.parentType = msg->parenttype;
    info.childIds = msg->childids;
    info.childTypes = msg->childtypes;
    info.nodename = msg->nodename;
    info.type = msg->type;
    info.machineip = msg->machineip;
    info.subscriber = msg->subscriber;
    //TODO check if all fields are used
    return info;
}


}
