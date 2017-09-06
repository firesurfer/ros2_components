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


namespace ros2_components
{
ComponentManager::ComponentManager(rclcpp::node::Node::SharedPtr _localNode, bool _runSychronous)
{
    using namespace std::placeholders;
    //Variable Assignments
    this->RosNode = _localNode;

    this->syncResponses = _runSychronous;
    //Start responder thread
    if(!_runSychronous)
        this->responder_thread = std::make_unique<std::thread>(std::bind(&ComponentManager::RespondingTask,this));
    //Qos Profile
    //rmw_qos_profile_services_default
    component_manager_profile = rmw_qos_profile_default;
    component_manager_profile.depth = 100;
    component_manager_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;


    //component_manager_profile.history = RMW_QOS_POLICY_KEEP_ALL_HISTORY;

    //Subscriptions
    this->ListComponentsResponseSubscription = RosNode->create_subscription<ros2_components_msg::msg::ListComponentsResponse>("ListComponentsResponse", std::bind(&ComponentManager::ListComponentsResponseCallback, this,_1), component_manager_profile);

    //Publishers
    this->ListComponentsRequestPublisher = RosNode->create_publisher<ros2_components_msg::msg::ListComponentsRequest>("ListComponentsRequest",component_manager_profile);
    std::srand(std::time(0)); // use current time as seed for random generator
    LOG(Info) << "Created new instance of a ComponentManager" << std::endl;
}

ComponentManager::ComponentManager(rclcpp::node::Node::SharedPtr _localNode, EntityBase::SharedPtr _baseEntity, bool _runSychronous): ComponentManager(_localNode, _runSychronous)
{

    //Variable Assignments
    this->BaseEntity = _baseEntity; //TODO register to components and add change callbacks
    for(EntityBase::SharedPtr & child:BaseEntity->getAllChilds())
    {
        connect(child.get(), &EntityBase::childAdded, this, &ComponentManager::OnChildAdded, Qt::DirectConnection);
        connect(child.get(), &EntityBase::childRemoved,this, &ComponentManager::OnChildRemoved, Qt::DirectConnection);
        connect(child.get(), &EntityBase::entityDeleted,this, &ComponentManager::OnEntityDeleted, Qt::DirectConnection);
    }
    //Subscriptions
    using namespace std::placeholders;
    this->ListComponentsRequestSubscription = RosNode->create_subscription<ros2_components_msg::msg::ListComponentsRequest>("ListComponentsRequest", std::bind(&ComponentManager::ListComponentsRequestCallback, this,_1), component_manager_profile);

    //Publishers
    this->ListComponentsResponsePublisher = RosNode->create_publisher<ros2_components_msg::msg::ListComponentsResponse>("ListComponentsResponse",component_manager_profile);

    this->updateTimer = RosNode->create_wall_timer(1s, std::bind(&ComponentManager::GenerateResponse,this));

}

ComponentManager::~ComponentManager()
{
    this->abort =true;
    if(responder_thread)
        responder_thread->join();

    LOG(Warning) << "Deletion of component manager. We interpret this that the whole cube got disposed. Advertising deletion!" << std::endl;
    std::function<void(EntityBase::SharedPtr)> iterateFunc = [&](EntityBase::SharedPtr ent)
    {
        ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::FromEntity(ent).toRosMessage();
        msg->nodename = RosNode->get_name();
        msg->deleted = true;
        this->ListComponentsResponsePublisher->publish(msg);
        for(EntityBase::SharedPtr & child:ent->getAllChilds())
        {

            iterateFunc(child);
        }
    };
    iterateFunc(BaseEntity);

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

int64_t ComponentManager::CountComponents()
{
    return Components.size();
}

std::vector<string> ComponentManager::ListNodes()
{
    return this->RosNode->get_node_graph_interface()->get_node_names();
}

std::vector<ComponentInfo> ComponentManager::ListComponentsBy(ComponentListFilter filter)
{
    return filter.Filter(this->Components);
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

void ComponentManager::UpdateComponentsList()
{
    ros2_components_msg::msg::ListComponentsRequest::SharedPtr request = std::make_shared<ros2_components_msg::msg::ListComponentsRequest>();
    request->nodename = RosNode->get_name();
    this->ListComponentsRequestPublisher->publish(request);
}

bool ComponentManager::CheckIfChildsAreAvailable(uint64_t id)
{
    std::function<bool(uint64_t)> checkChilds = [&](uint64_t id) ->bool
    {
        bool success = false;
        ComponentInfo info = this->GetInfoToId(id,&success);
        if(!success)
            return false;
        for(int64_t & childId : info.childIds )
        {
            bool result = checkChilds(childId);
            return result;
        }
        return true;
    };
    return checkChilds(id);
}

void ComponentManager::DisableThreadedResponse()
{
    abort = true;
    syncResponses = true;

}

void ComponentManager::ListComponentsRequestCallback(ros2_components_msg::msg::ListComponentsRequest::SharedPtr msg)
{
    if(RosNode->get_name() != msg->nodename)
    {
        if(!syncResponses)
            triggerResponseQueue.push(true);
        else
        {
            GenerateResponse();
        }
    }
}

void ComponentManager::ListComponentsResponseCallback(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg)
{
    if(RosNode->get_name() != msg->nodename)
    {
        bool foundInList = false;
        bool toDelete = false;
        ComponentInfo currentInfo = ComponentInfoFactory::FromListComponentsResponseMessage(msg);
        for(ComponentInfo & myInfo: Components)
        {

            if(myInfo.name == currentInfo.name)
            {
                //LOG(Debug) << "My: " << myInfo.id << " " << myInfo.name << " Remote: " << currentInfo.id << " " << currentInfo.name << std::endl;
                foundInList = true;
                if(!msg->deleted)
                {
                    //TODO implement comparison for component info
                    myInfo = currentInfo;
                    emit ComponentChanged(myInfo);

                }
                else
                {

                    toDelete = true;
                    //TODO delete from list more efficient
                    emit ComponentDeleted(myInfo);

                }
                break;
            }
        }

        if(!foundInList)
        {
            Components.push_back(currentInfo);
            emit NewComponentFound(currentInfo);
        }
        if(toDelete)
        {
            size_t pos = 0;
            for(auto & info: Components)
            {
                if( info.id == msg->id)
                {
                    break;
                }
                pos++;
            }
            Components.erase(Components.begin()+pos);
        }
    }

}





void ComponentManager::GenerateResponse()
{
    auto responseFunc = [&]()
    {
        if(BaseEntity)
        {
            ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::FromEntity(this->BaseEntity).toRosMessage();
            msg->nodename = RosNode->get_name();
            msg->deleted = false;
            this->ListComponentsResponsePublisher->publish(msg);
            std::function<void(EntityBase::SharedPtr)> iteratingFunc = [&](EntityBase::SharedPtr ent)
            {
                auto respMsg = ComponentInfoFactory::FromEntity(ent).toRosMessage();
                respMsg->nodename = RosNode->get_name();
                respMsg->deleted = false;
                this->ListComponentsResponsePublisher->publish(respMsg);
            };

            this->BaseEntity->IterateThroughAllChilds(iteratingFunc);
        }
    };

    responseFunc();
}

void ComponentManager::RespondingTask()
{
    int count = 0;
    while(!abort)
    {

        while(triggerResponseQueue.empty() && !abort)
        {
            if(abort)
                return;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            count ++;
            if(count > 100)
            {
                count = 0;
                GenerateResponse();
            }
        }
        if(abort)
            return;
        while(!triggerResponseQueue.empty())
        {
            triggerResponseQueue.pop();
        }
        GenerateResponse();
    }
}

void ComponentManager::OnChildAdded(EntityBase::SharedPtr child, EntityBase::SharedPtr parent, bool remote)
{
    LOG(Info) << "Child added: " << child->getName() << std::endl;
    //Connect to add events
    connect(child.get(),&EntityBase::childAdded,this, &ComponentManager::OnChildAdded);
    connect(child.get(),&EntityBase::childRemoved,this, &ComponentManager::OnChildRemoved);
    connect(child.get(), &EntityBase::entityDeleted,this, &ComponentManager::OnEntityDeleted);
    //Publish component information
    ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::FromEntity(child).toRosMessage();
    msg->nodename = RosNode->get_name();
    msg->deleted = false;
    this->ListComponentsResponsePublisher->publish(msg);

}

void ComponentManager::OnChildRemoved(EntityBase::SharedPtr child, EntityBase::SharedPtr parent, bool remote)
{
    LOG(Info) << "Child removed: " << child->getName() << std::endl;
    disconnect(child.get(), &EntityBase::childAdded, this, &ComponentManager::OnChildAdded);
    disconnect(child.get(), &EntityBase::childRemoved, this, &ComponentManager::OnChildRemoved);

    ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::FromEntity(child).toRosMessage();
    msg->nodename = RosNode->get_name();
    msg->deleted = true;
    this->ListComponentsResponsePublisher->publish(msg);
}

void ComponentManager::OnEntityDeleted(ComponentInfo info)
{
    LOG(Warning) << info.name << " got deleted" << std::endl;
    ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = info.toRosMessage();
    msg->nodename = RosNode->get_name();
    msg->deleted = true;
    this->ListComponentsResponsePublisher->publish(msg);
}





}

