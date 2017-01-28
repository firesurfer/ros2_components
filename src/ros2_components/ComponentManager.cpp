/*
 * Copyright 2016 <Lennart Nachtigall> <firesurfer65@yahoo.de>
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

#include "ComponentManager.h"
#include "QVariant"

namespace ros2_components
{
ComponentManager::ComponentManager(rclcpp::node::Node::SharedPtr _localNode, EntityBase::SharedPtr _baseEntity)
{
    LOG(Info) << "Created new instance of a ComponentManager" << std::endl;
    this->localNode = _localNode;
    using namespace std::placeholders;
    //rmw_qos_profile_services_default
    rmw_qos_profile_t component_manager_profile = rmw_qos_profile_parameters;
    component_manager_profile.depth = 1000;
    //component_manager_profile.history = RMW_QOS_POLICY_KEEP_ALL_HISTORY;
    this->AdvertisementSubscription = localNode->create_subscription<ros2_components_msg::msg::EntityAdvertisement>("EntityAdvertisement", std::bind(&ComponentManager::AdvertisementCallback, this,_1), component_manager_profile);
    std::srand(std::time(0)); // use current time as seed for random generator
}

bool ComponentManager::IDAlreadyInUse(uint64_t id)
{
    for(auto & myInfo: Components)
    {
        if(myInfo.id == id)
            return true;
    }
    return false;
}

std::vector<ComponentInfo> ComponentManager::ListComponents()
{
    return Components;
}

ComponentInfo ComponentManager::GetInfoToId(uint64_t id, bool *success)
{
    if(success != NULL)
        *success = false;
    for(auto & comp  : Components)
    {
        if(id == comp.id)
        {
            if(success != NULL)
                *success= true;
            return comp;
        }
    }
    ComponentInfo dummy;
    return dummy;
}

void ComponentManager::ProcessNewAdvertisment(const ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg, ComponentInfo info)
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

void ComponentManager::ProcessChangeAdvertisment(const ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg, ComponentInfo info)
{
    LOG(Debug) << "Got type " <<"Change"<<" advertisement:" << msg->id << " " << info.name << " " << msg->type << std::endl;
    int i = 0;
    bool found = false;
    for(auto & compinfo : Components)
    {
        if(compinfo.id == msg->id)
        {
            Components[i] = info;
            found = true;
            emit ComponentChanged(info);
            break;
        }
        i++;
    }
    if(!found)
    {
        LOG(Warning) << "Got change advertisement without new advertisement before: " << msg->id << " " << info.name << " " << "Interpreting as new advertisement"<< std::endl;
        ProcessNewAdvertisment(msg,info);
    }
}

void ComponentManager::ProcessDeleteAdvertisment(const ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg, ComponentInfo info)
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
        ProcessNewAdvertisment(msg,info);
    }
    else if((AdvertisementType::Enum)msg->advertisementtype == AdvertisementType::Enum::Change)
    {

        ProcessChangeAdvertisment(msg,info);
    }
    else if((AdvertisementType::Enum)msg->advertisementtype == AdvertisementType::Enum::Delete)
    {
       ProcessDeleteAdvertisment(msg,info);
    }

}

//void EntityBase::Advertise(AdvertisementType::Enum type)
//{
//    if(!advertised && type != AdvertisementType::Enum::New)
//        return;
//    if(advertised && type == AdvertisementType::Enum::New)
//        return;
//    if(this->advertisementPublisher != NULL)
//    {
//        LOG(Debug) << "Advertising:" << getName()<< " Type:" << type << std::endl;

//        advertised = true;
//        ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg = std::make_shared<ros2_components_msg::msg::EntityAdvertisement>();

//        int64_t ipAddr =0;
//        foreach(const QNetworkInterface &interface, QNetworkInterface::allInterfaces())
//        {
//            if(!interface.name().contains("vmnet"))
//            {
//                foreach (const QHostAddress &address, interface.allAddresses())
//                {
//                    if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
//                    {
//                        //LOG(Debug) << "Ip address is:" << address.toString().toStdString() << std::endl;
//                        if(!address.isLoopback())
//                        {
//                            ipAddr = address.toIPv4Address();
//                            break;
//                        }
//                    }
//                }
//            }
//        }

//        msg->nodename = this->parentNode->get_name();
//        msg->advertisementtype = (int)type;
//        msg->id = getId();
//        msg->machineip = ipAddr;
//        if(getParent() != NULL)
//        {
//            msg->parent = getParent()->getId();
//            msg->parenttype = getParent()->getClassName();
//        }
//        else
//        {
//            msg->parent = -1;
//            msg->parenttype = "";
//        }
//        msg->type = this->className;
//        builtin_interfaces::msg::Time time;
//        simpleLogger::set_now(time);
//        msg->stamp = time;


//        for(auto & child: childs)
//        {
//            msg->childtypes.push_back(child->getClassName());
//            msg->childids.push_back(child->getId());

//        }


//        msg->componentname = getName();
//        LOG(Debug) << "Publishing advertisementmessage in " << getName() << " now" << std::endl;
//        advertisementPublisher->publish(msg);

//    }
//}

}

