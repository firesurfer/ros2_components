#include "ComponentManager.h"
#include "QVariant"

namespace ros2_components
{
ComponentManager::ComponentManager(rclcpp::node::Node::SharedPtr _localNode)
{
    LOG(Info) << "Created new instance of a ComponentManager" << std::endl;
    this->localNode = _localNode;
    using namespace std::placeholders;
    this->AdvertisementSubscription = localNode->create_subscription<ros2_components_msg::msg::EntityAdvertisement>("EntityAdvertisement", std::bind(&ComponentManager::AdvertisementCallback, this,_1), rmw_qos_profile_default);
    std::srand(std::time(0)); // use current time as seed for random generator
}

void ComponentManager::AdvertisementCallback(const ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg)
{
    //Any Advertisement message that gets caught will be processed in this function
    if(msg->nodename == this->localNode->get_name())
        return;
    ComponentInfo info;
    info.nodename = msg->nodename;
    info.childIds = msg->childids;
    info.childTypes = msg->childtypes;
    info.id = msg->id;
    info.type = msg->type;
    info.name = msg->componentname;
    info.parentId = msg->parent;
    info.machineip = msg->machineip;

    LOG(Debug) << "Got new advertisement: " << info.id << std::endl;
    //TODO Replace if with switch statement
    if((AdvertisementType::Enum)msg->advertisementtype == AdvertisementType::Enum::New)
    {
        LOG(Debug) << "Got type " <<"New"<<" advertisement:" << msg->id << " " << info.name << " " << msg->type << std::endl;
        for(auto & myInfo: Components)
        {
            if(myInfo.id == info.id)
            {
                if(myInfo.nodename == info.nodename)
                {
                    LOG(Warning) << "Id already used in system but was advertised from the same node - I assume that the node was restarted - I'm going to interpret this advertisement as change ad" << std::endl;
                    LOG(Debug) << "Got type " <<"Change"<<" advertisement:" << msg->id << " " << info.name << " " << msg->type << std::endl;
                    int i = 0;
                    for(auto & compinfo : Components)
                    {
                        if(compinfo.id == msg->id)
                        {
                            Components[i] = info;
                            emit ComponentChanged(info);
                            break;
                        }
                        i++;
                    }
                    return;
                }
                else
                {
                    LOG(Fatal) << "Id already used in system -> this might result in unpredictable behaviour - I'm not going to interpret this ad: "<< info.name << " ID is used by: " <<myInfo.name<< std::endl;
                    return;

                }
            }
        }
        Components.push_back(info);

        emit NewComponentAvailable(info);
    }
    else if((AdvertisementType::Enum)msg->advertisementtype == AdvertisementType::Enum::Change)
    {
        LOG(Debug) << "Got type " <<"Change"<<" advertisement:" << msg->id << " " << info.name << " " << msg->type << std::endl;
        int i = 0;
        for(auto & compinfo : Components)
        {
            if(compinfo.id == msg->id)
            {
                Components[i] = info;
                emit ComponentChanged(info);
                break;
            }
            i++;
        }
    }
    else if((AdvertisementType::Enum)msg->advertisementtype == AdvertisementType::Enum::Delete)
    {
        LOG(Debug) << "Got type " <<"Delete"<<" advertisement:" << msg->id << " " << info.name << " " << msg->type << std::endl;
        int i = 0;
        for(auto & compinfo : Components)
        {
            if(compinfo.id == msg->id)
            {
                Components[i] = info;
                break;
            }
            i++;
        }
        Components.erase(Components.begin()+i);
        emit ComponentDeleted(info);
    }

}

}
