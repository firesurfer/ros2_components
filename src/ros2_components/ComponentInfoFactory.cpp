#include "ComponentInfoFactory.h"
namespace ros2_components
{
ComponentInfoFactory::ComponentInfoFactory()
{

}

ComponentInfo ComponentInfoFactory::FromEntity(EntityBase::SharedPtr ent)
{
    ComponentInfo info;
    info.name = ent->getParentNode();
    info.id = ent->getId();

    //TODO move this into a more general place
    int64_t ipAddr =0;
    foreach(const QNetworkInterface &interface, QNetworkInterface::allInterfaces())
    {
        if(!interface.name().contains("vmnet"))
        {
            foreach (const QHostAddress &address, interface.allAddresses())
            {
                if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
                {
                    //LOG(Debug) << "Ip address is:" << address.toString().toStdString() << std::endl;
                    if(!address.isLoopback())
                    {
                        ipAddr = address.toIPv4Address();
                        break;
                    }
                }
            }
        }
    }
    info.machineip = ipAddr;
    if(ent->getParent() != NULL)
    {
        info.parentId = ent->getParent()->getId();
        info.parentId = ent->getParent()->getClassName();
    }
    else
    {
        msg->parent = -1;
        msg->parenttype = "";
    }
    info.type = ent->getClassName();

    for(EntityBase::SharedPtr & child : ent->getAllChilds())
    {
        info.childIds.push_back(child->getId());
        info.childTypes.push_back(child->getClassName());
    }
    info.name = ent->getName();
    return info;

}

ComponentInfo ComponentInfoFactory::FromListComponentsResponseMessage(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg)
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
    //TODO check if all fields are used
    return info;
}
}
