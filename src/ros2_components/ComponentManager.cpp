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
ComponentManager::ComponentManager(rclcpp::node::Node::SharedPtr _localNode) : componentsReader(0)
{
    using namespace std::placeholders;
    //Variable Assignments
    this->RosNode = _localNode;

    //Start responder thread
    //Qos Profile
    //rmw_qos_profile_services_default
    component_manager_profile = rmw_qos_profile_default;
    component_manager_profile.depth = 100;
    component_manager_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    component_manager_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    //component_manager_profile.history = RMW_QOS_POLICY_KEEP_ALL_HISTORY;

    //Subscriptions
    this->ListComponentsResponseSubscription = RosNode->create_subscription<ros2_components_msg::msg::ListComponentsResponse>("ListComponentsResponse", std::bind(&ComponentManager::ListComponentsResponseCallback, this,_1), component_manager_profile);

    //Publishers
    this->ListComponentsRequestPublisher = RosNode->create_publisher<ros2_components_msg::msg::ListComponentsRequest>("ListComponentsRequest",component_manager_profile);
    std::srand(std::time(0)); // use current time as seed for random generator
    LOG(Info) << "Created new instance of a ComponentManager" << std::endl;
}

ComponentManager::ComponentManager(rclcpp::node::Node::SharedPtr _localNode, EntityBase::SharedPtr _baseEntity): ComponentManager(_localNode)
{

    //Variable Assignments
    this->BaseEntity = _baseEntity;

    //Connect to entity structure change callbacks
    auto func = [&](EntityBase::SharedPtr child)
    {
        connect(child.get(), &EntityBase::childAdded, this, &ComponentManager::OnChildAdded, Qt::DirectConnection);
        connect(child.get(), &EntityBase::childRemoved,this, &ComponentManager::OnChildRemoved, Qt::DirectConnection);
        connect(child.get(), &EntityBase::entityDeleted,this, &ComponentManager::OnEntityDeleted, Qt::DirectConnection);
    };
    connect(BaseEntity.get(), &EntityBase::childAdded, this, &ComponentManager::OnChildAdded, Qt::DirectConnection);
    connect(BaseEntity.get(), &EntityBase::childRemoved,this, &ComponentManager::OnChildRemoved, Qt::DirectConnection);
    connect(BaseEntity.get(), &EntityBase::entityDeleted,this, &ComponentManager::OnEntityDeleted, Qt::DirectConnection);
    //Call lambda
    BaseEntity->iterateThroughAllChilds(func);

    //Subscriptions
    using namespace std::placeholders;
    this->ListComponentsRequestSubscription = RosNode->create_subscription<ros2_components_msg::msg::ListComponentsRequest>("ListComponentsRequest", std::bind(&ComponentManager::ListComponentsRequestCallback, this,_1), component_manager_profile);

    //Publishers
    this->ListComponentsResponsePublisher = RosNode->create_publisher<ros2_components_msg::msg::ListComponentsResponse>("ListComponentsResponse",component_manager_profile);
   // this->updateTimer = RosNode->create_wall_timer(1s, std::bind(&ComponentManager::GenerateResponse,this));


}

ComponentManager::~ComponentManager()
{
    this->BaseEntity.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    LOG(Warning) << "Deletion of component manager" << std::endl;
}

std::vector<ComponentInfo> ComponentManager::ListComponents()
{
    ReaderGuard rg(this);
    return Components;
}

std::vector<string> ComponentManager::ListNodes()
{
    return this->RosNode->get_node_graph_interface()->get_node_names();
}

std::vector<ComponentInfo> ComponentManager::ListComponentsBy(std::function<bool(const ComponentInfo&)> filter)
{
    std::vector<ComponentInfo> ret;
    {
        ReaderGuard rg(this);
        for (const auto & comp : Components)
        {
            if (filter(comp))
            {
                ret.emplace_back(comp);
            }
        }
    }
    return ret;
}

ComponentInfo ComponentManager::GetInfoWithFilter(std::function<bool (const ComponentInfo &)> filter, bool *success, std::chrono::milliseconds timeout)
{
    auto startTime = std::chrono::system_clock::now();
    bool waitIndefinitely = timeout < std::chrono::milliseconds::zero();
    ComponentInfo relevantInfo;
    bool found = false;
    {
        ReaderGuard rg(this);
        for(const auto & comp : Components)
        {
            if(filter(comp))
            {
                relevantInfo = comp;
                found = true;
                break;
            }
        }
    }
    if (!found)
    {
        auto newComponentFunc = [filter, &relevantInfo, &found] (ComponentInfo info) -> void
        {
            if (filter(info))
            {
                relevantInfo = info;
                found = true;
            }
        };

        //Disconnects the connection when it goes out of scope
        auto auto_disconnect = [] (QMetaObject::Connection* con) { QObject::disconnect(*con); delete con; };
        std::unique_ptr<QMetaObject::Connection, decltype(auto_disconnect)> connection(new QMetaObject::Connection(QObject::connect(this, &ComponentManager::NewComponentFound, newComponentFunc)), auto_disconnect);

        while (!found && rclcpp::ok())
        {
            long hertz = 80L;
            if (!waitIndefinitely)
            {
                auto diffTime = startTime + timeout - std::chrono::system_clock::now();
                if (diffTime <= std::chrono::milliseconds::zero())
                {
                    break;
                }
                hertz = std::max(hertz, (long) (std::chrono::milliseconds(1000).count() / diffTime.count()));
            }

            rclcpp::WallRate loop_rate(hertz);
            rclcpp::spin_some(RosNode); //can throw exception FIXME rethrow with more descriptive exception?
            loop_rate.sleep();
        }
    }
    if (success != nullptr)
    {
        *success = found;
    }
    return relevantInfo;
}

ComponentInfo ComponentManager::GetInfoToId(int64_t id, bool *success, std::chrono::milliseconds timeout)
{
    auto filter = [id] (const ComponentInfo& info) -> bool
    {
        return info.id == id;
    };
    return GetInfoWithFilter(filter, success, timeout);
}

void ComponentManager::GetInfoWithFilterAsync(std::function<void(ComponentInfo)> callback, std::function<bool(const ComponentInfo&)> filter, std::chrono::milliseconds timeout)
{
    auto startTime = std::chrono::system_clock::now();

    //Impersonating a Reader prevents the NewComponentFound signal to be sent and thus the callback to be called until ready
    ReaderGuard rg(this);

    bool found = false;
    ComponentInfo info = GetInfoWithFilter(filter, &found);
    if (found)
    {
        callback(info);
    }
    else
    {
        std::unique_lock<std::mutex> lck(callbacksMutex);
        callbacks.push_front(QMetaObject::Connection());
        auto callback_it = callbacks.begin();
        auto full_callback = [callback, filter, startTime, timeout, this, callback_it] (ComponentInfo comp)
        {
            bool found = false;
            bool timedOut = false;
            if (timeout > std::chrono::milliseconds::zero() && std::chrono::system_clock::now() - timeout >= startTime)
            {
                timedOut = true;
                callback(ComponentInfo());
            }
            else
            {
                if (filter(comp))
                {
                    found = true;
                    callback(comp);
                }
            }
            if (found || timedOut)
            {
                std::unique_lock<std::mutex> lck(callbacksMutex);
                QObject::disconnect(*callback_it);
                this->callbacks.erase(callback_it);
            }
        };
        *callback_it = QObject::connect(this, &ComponentManager::NewComponentFound, full_callback);
    }
}

void ComponentManager::GetInfoToIdAsync(std::function<void (ComponentInfo)> callback, int64_t id, std::chrono::milliseconds timeout)
{
    auto filter = [id] (const ComponentInfo& info) -> bool
    {
        return info.id == id;
    };
    return GetInfoWithFilterAsync(callback, filter, timeout);
}

void ComponentManager::UpdateComponentsList()
{
    ros2_components_msg::msg::ListComponentsRequest::SharedPtr request = std::make_shared<ros2_components_msg::msg::ListComponentsRequest>();
    request->nodename = RosNode->get_name();
    this->ListComponentsRequestPublisher->publish(request);
}

void ComponentManager::ListComponentsRequestCallback(ros2_components_msg::msg::ListComponentsRequest::SharedPtr msg)
{
    if(RosNode->get_name() != msg->nodename)
    {
        GenerateResponse();
    }
}

void ComponentManager::ListComponentsResponseCallback(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg)
{
    if(RosNode->get_name() != msg->nodename)
    {
        bool foundInList = false;
        bool toDelete = false;
        ComponentInfo currentInfo = ComponentInfoFactory::fromListComponentsResponseMessage(msg);
        {
            ReaderGuard rg(this);
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
                          LOG(Info) << "Deleting: " << myInfo.name << std::endl;
                        emit ComponentDeleted(myInfo);

                    }
                    break;
                }
            }
        }

        {
            //Writing to Components
            std::unique_lock<std::mutex> lck(componentsMutex);
            while (componentsReader > 0)
            {
                componentsCV.wait(lck);
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
}

void ComponentManager::GenerateResponse()
{
    auto responseFunc = [&]()
    {
        if(BaseEntity)
        {
            ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::fromEntity(this->BaseEntity).toRosMessage();
            msg->nodename = RosNode->get_name();
            msg->deleted = false;
            this->ListComponentsResponsePublisher->publish(msg);
            std::function<void(EntityBase::SharedPtr)> iteratingFunc = [&](EntityBase::SharedPtr ent)
            {
                auto respMsg = ComponentInfoFactory::fromEntity(ent).toRosMessage();
                respMsg->nodename = RosNode->get_name();
                respMsg->deleted = false;
                this->ListComponentsResponsePublisher->publish(respMsg);
            };

            this->BaseEntity->iterateThroughAllChilds(iteratingFunc);
        }
    };

    responseFunc();
}

void ComponentManager::OnChildAdded(EntityBase::SharedPtr child, EntityBase::SharedPtr parent, bool remote)
{
    LOG(Info) << "Child added: " << child->getName() << std::endl;
    //Connect to add events
    connect(child.get(),&EntityBase::childAdded,this, &ComponentManager::OnChildAdded);
    connect(child.get(),&EntityBase::childRemoved,this, &ComponentManager::OnChildRemoved);
    connect(child.get(), &EntityBase::entityDeleted,this, &ComponentManager::OnEntityDeleted);
    //Publish component information
    ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::fromEntity(child).toRosMessage();
    msg->nodename = RosNode->get_name();
    msg->deleted = false;
    this->ListComponentsResponsePublisher->publish(msg);

}

void ComponentManager::OnChildRemoved(EntityBase::SharedPtr child, EntityBase::SharedPtr parent, bool remote)
{
    LOG(Info) << "Child removed: " << child->getName() << std::endl;
    disconnect(child.get(), &EntityBase::childAdded, this, &ComponentManager::OnChildAdded);
    disconnect(child.get(), &EntityBase::childRemoved, this, &ComponentManager::OnChildRemoved);

    ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::fromEntity(child).toRosMessage();
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

ComponentManager::ReaderGuard::ReaderGuard(ComponentManager* comp) : cm(comp)
{
    std::unique_lock<std::mutex> lck(cm->componentsMutex);
    cm->componentsReader++;
}

ComponentManager::ReaderGuard::~ReaderGuard()
{
    {
        std::unique_lock<std::mutex> lck(cm->componentsMutex);
        cm->componentsReader--;
    }
    cm->componentsCV.notify_all();
}

}
