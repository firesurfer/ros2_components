#include "ComponentInfoFactory.h"
namespace ros2_components
{
ComponentInfoFactory::ComponentInfoFactory()
{

}

ComponentInfo ComponentInfoFactory::FromEntity(EntityBase::SharedPtr ent)
{
    ComponentInfo info;
    info.name = ent->getName();
    info.id = ent->getId();

    //TODO move this into a more general place

    info.machineip = GetLocalIpV4();
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
    info.subscriber = msg->subscriber;
    //TODO check if all fields are used
    return info;
}

ComponentInfo ComponentInfoFactory::FromComponentChangedMessage(ros2_components_msg::msg::ComponentChanged::SharedPtr msg)
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

int64_t ComponentInfoFactory::GetLocalIpV4()
{
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
    return ipAddr;
}

std::vector<uint8_t> ComponentInfoFactory::GetLocalIpV6()
{
    std::vector<uint8_t> returnAddr;
    foreach(const QNetworkInterface &interface, QNetworkInterface::allInterfaces())
    {
        if(!interface.name().contains("vmnet"))
        {
            foreach (const QHostAddress &address, interface.allAddresses())
            {
                if (address.protocol() == QAbstractSocket::IPv6Protocol && address != QHostAddress(QHostAddress::LocalHost))
                {
                    //LOG(Debug) << "Ip address is:" << address.toString().toStdString() << std::endl;
                    if(!address.isLoopback())
                    {
                        Q_IPV6ADDR ipAddr = address.toIPv6Address();
                        for(int i = 0; i < 16;i++)
                        {
                            returnAddr.push_back(ipAddr[i]);
                        }
                        break;
                    }
                }
            }
        }
    }
    return returnAddr;
}
}
