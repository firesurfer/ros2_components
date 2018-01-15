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
ComponentManager::ComponentManager(rclcpp::Node::SharedPtr _localNode) : componentsReader(0)
{
    using namespace std::placeholders;
    //Variable Assignments
    this->rosNode = _localNode;

    //Start responder thread
    //Qos Profile
    //rmw_qos_profile_services_default
    component_manager_profile = rmw_qos_profile_default;
    component_manager_profile.depth = 2000;
    component_manager_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    component_manager_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    //component_manager_profile.history = RMW_QOS_POLICY_KEEP_ALL_HISTORY;

    //Subscriptions
    this->listComponentsResponseSubscription = rosNode->create_subscription<ros2_components_msg::msg::ListComponentsResponse>("listComponentsResponse", std::bind(&ComponentManager::listComponentsResponseCallback, this,_1), component_manager_profile);

    //Publishers
    this->listComponentsRequestPublisher = rosNode->create_publisher<ros2_components_msg::msg::ListComponentsRequest>("listComponentsRequest",component_manager_profile);

    LOG(Info) << "Created new instance of a ComponentManager" << std::endl;


    //Are only used when Components are registered, instantiate them here due to issue: https://github.com/ros2/rmw_fastrtps/issues/157
    this->listComponentsRequestSubscription = rosNode->create_subscription<ros2_components_msg::msg::ListComponentsRequest>("listComponentsRequest", std::bind(&ComponentManager::listComponentsRequestCallback, this,_1), component_manager_profile);
    this->listComponentsResponsePublisher = rosNode->create_publisher<ros2_components_msg::msg::ListComponentsResponse>("listComponentsResponse",component_manager_profile);
}

ComponentManager::~ComponentManager()
{
    this->baseEntity.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    LOG(Warning) << "Deletion of component manager" << std::endl;
}

void ComponentManager::registerComponents(EntityBase::SharedPtr _baseEntity)
{
    //Variable Assignments
    while (_baseEntity->getParent() != nullptr)
    {
        _baseEntity = _baseEntity->getParent();
    }
    this->baseEntity = _baseEntity;

    //Connect to entity structure change callbacks
    auto func = [&](EntityBase::SharedPtr child)
    {
        connect(child.get(), &EntityBase::childAdded, this, &ComponentManager::onChildAdded, Qt::DirectConnection);
        connect(child.get(), &EntityBase::childRemoved,this, &ComponentManager::onChildRemoved, Qt::DirectConnection);
        connect(child.get(), &EntityBase::entityDeleted,this, &ComponentManager::onEntityDeleted, Qt::DirectConnection);
    };
    connect(baseEntity.get(), &EntityBase::childAdded, this, &ComponentManager::onChildAdded, Qt::DirectConnection);
    connect(baseEntity.get(), &EntityBase::childRemoved,this, &ComponentManager::onChildRemoved, Qt::DirectConnection);
    connect(baseEntity.get(), &EntityBase::entityDeleted,this, &ComponentManager::onEntityDeleted, Qt::DirectConnection);
    //Call lambda
    baseEntity->iterateThroughAllChilds(func);

    //Tell other nodes about entities
    generateResponse();
}

std::vector<ComponentInfo> ComponentManager::listComponents()
{
    ReaderGuard rg(this);
    return components;
}

std::vector<ComponentInfo> ComponentManager::listNodes()
{
    //return this->rosNode->get_node_graph_interface()->get_node_names();
    std::vector<ComponentInfo> node_infos;
    for(ComponentInfo& info: components)
    {
        if(info.type == "NodeEntity")
            node_infos.push_back(info);
    }
    return node_infos;
}

std::vector<ComponentInfo> ComponentManager::listComponentsBy(std::function<bool(const ComponentInfo&)> filter)
{
    std::vector<ComponentInfo> ret;
    {
        ReaderGuard rg(this);
        for (const auto & comp : components)
        {
            if (filter(comp))
            {
                ret.emplace_back(comp);
            }
        }
    }
    return ret;
}

ComponentInfo ComponentManager::getInfoWithFilter(std::function<bool (const ComponentInfo &)> filter, bool *success, std::chrono::milliseconds timeout)
{
    auto startTime = std::chrono::system_clock::now();
    bool waitIndefinitely = timeout < std::chrono::milliseconds::zero();
    ComponentInfo relevantInfo;
    bool found = false;
    {
        ReaderGuard rg(this);
        for(const auto & comp : components)
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
        std::unique_ptr<QMetaObject::Connection, decltype(auto_disconnect)> connection(new QMetaObject::Connection(QObject::connect(this, &ComponentManager::newComponentFound, newComponentFunc)), auto_disconnect);

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
            try
            {
                rclcpp::spin_some(rosNode);
            }
            catch (std::runtime_error e)
            {
                throw AlreadySpinningException();
            }
            loop_rate.sleep();
        }
    }
    if (success != nullptr)
    {
        *success = found;
    }
    return relevantInfo;
}

ComponentInfo ComponentManager::getInfoToId(int64_t id, bool *success, std::chrono::milliseconds timeout)
{
    auto filter = [id] (const ComponentInfo& info) -> bool
    {
        return info.id == id;
    };
    return getInfoWithFilter(filter, success, timeout);
}

void ComponentManager::getInfoWithFilterAsync(std::function<void(ComponentInfo)> callback, std::function<bool(const ComponentInfo&)> filter, std::chrono::milliseconds timeout)
{
    auto startTime = std::chrono::system_clock::now();

    //Impersonating a Reader prevents this to miss a newComponentFound signal after checking already existing components
    ReaderGuard rg(this);

    bool found = false;
    ComponentInfo info = getInfoWithFilter(filter, &found);
    if (found)
    {
        callback(info);
    }
    else
    {
        std::unique_lock<std::mutex> lck(callbacksMutex);
        callbacks.push_front(QMetaObject::Connection());
        auto callback_it = callbacks.begin();

        std::shared_ptr<std::mutex> timeoutMutex = std::make_shared<std::mutex>();
        std::shared_ptr<bool> finished = std::make_shared<bool>(false);
        auto full_callback = [callback, filter, this, callback_it, finished, timeoutMutex] (ComponentInfo comp)
        {
            std::unique_lock<std::mutex> tLck(*timeoutMutex);
            if (*finished)
            {
                return;
            }
            if (filter(comp))
            {
                *finished = true;
                callback(comp);
                std::unique_lock<std::mutex> lck(callbacksMutex);
                QObject::disconnect(*callback_it);
                this->callbacks.erase(callback_it);
            }
        };
        *callback_it = QObject::connect(this, &ComponentManager::newComponentFound, full_callback);

        if (timeout >= std::chrono::milliseconds::zero())
        {
            auto timeout_handler = [callback, timeout, startTime, this, callback_it, finished, timeoutMutex] ()
            {
                std::this_thread::sleep_for(timeout - (std::chrono::system_clock::now() - startTime));
                std::unique_lock<std::mutex> tLck(*timeoutMutex);
                if (*finished)
                {
                    return;
                }
                else
                {
                    *finished = true;
                    callback(ComponentInfo());
                    std::unique_lock<std::mutex> lck(callbacksMutex);
                    QObject::disconnect(*callback_it);
                    this->callbacks.erase(callback_it);
                }
            };
            std::thread timer(timeout_handler);
            timer.detach();
        }
    }
}

void ComponentManager::getInfoToIdAsync(std::function<void (ComponentInfo)> callback, int64_t id, std::chrono::milliseconds timeout)
{
    auto filter = [id] (const ComponentInfo& info) -> bool
    {
        return info.id == id;
    };
    return getInfoWithFilterAsync(callback, filter, timeout);
}

void ComponentManager::updateComponentsList()
{
    ros2_components_msg::msg::ListComponentsRequest::SharedPtr request = std::make_shared<ros2_components_msg::msg::ListComponentsRequest>();
    request->nodename = rosNode->get_name();
    this->listComponentsRequestPublisher->publish(request);
}

std::shared_ptr<EntityBase> ComponentManager::rebuildComponent(const ComponentInfo & info, bool rebuildHierarchy, bool forcePubOrSubChange, std::chrono::milliseconds timeout)
{
    auto startTime = std::chrono::system_clock::now();
    bool waitIndefinitely = timeout < std::chrono::milliseconds::zero();
    if(!EntityFactory::contains(info.type))
    {
        throw EntityNotRegisteredException();
    }

    QGenericArgument subscribeArg;
    QGenericArgument idArg = Q_ARG(int64_t, info.id);

    if((!info.subscriber) != forcePubOrSubChange)
        subscribeArg = Q_ARG(bool, true);
    else
        subscribeArg = Q_ARG(bool, false);
    QGenericArgument nodeArg  =Q_ARG(std::shared_ptr< rclcpp::Node >, rosNode);

    std::shared_ptr<EntityBase> ent = EntityFactory::createInstanceFromName(info.type,idArg,subscribeArg,nodeArg);
    //std::shared_ptr<T> secEnt = dynamic_pointer_cast<T>(ent);

    //The following line recursively rebuild the tree structure that was published before
    if(rebuildHierarchy)
    {
        std::function<void(EntityBase::SharedPtr, ComponentInfo)> rec_build = [&](EntityBase::SharedPtr parentEntity, ComponentInfo parentInfo)
        {
            for(auto & child_id: parentInfo.childIds)
            {
                ComponentInfo childInfo;
                bool found_child = false;
                if (waitIndefinitely)
                {
                    childInfo = getInfoToId(child_id, &found_child, timeout);
                }
                else
                {
                    auto remaining_timeout = startTime + timeout - std::chrono::system_clock::now();
                    if (remaining_timeout < std::chrono::milliseconds::zero())
                    {
                        remaining_timeout = std::chrono::milliseconds::zero();
                    }
                    childInfo = getInfoToId(child_id, &found_child, std::chrono::duration_cast<std::chrono::milliseconds>(remaining_timeout));
                }
                if(!found_child)
                {
                    throw TimeoutException();
                }
                idArg = Q_ARG(int64_t, childInfo.id);

                //LOG(Debug) << "Childinfo : subscriber: " << childInfo.subscriber << std::endl;
                if(!childInfo.subscriber)
                    subscribeArg = Q_ARG(bool, true);
                else
                    subscribeArg = Q_ARG(bool, false);
                std::shared_ptr<EntityBase> child_obj = EntityFactory::createInstanceFromName(childInfo.type,idArg,subscribeArg,nodeArg);
                parentEntity->addChild(child_obj);
                rec_build(child_obj, childInfo);
            }
        };
        std::shared_ptr<EntityBase> parentEnt = dynamic_pointer_cast<EntityBase>(ent);
        rec_build(parentEnt,info);
    }
    ent->makeVirtual();
    return ent;
}

void ComponentManager::listComponentsRequestCallback(ros2_components_msg::msg::ListComponentsRequest::SharedPtr msg)
{
    if(rosNode->get_name() != msg->nodename)
    {
        generateResponse();
    }
}

void ComponentManager::listComponentsResponseCallback(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg)
{
    if(rosNode->get_name() != msg->nodename)
    {
        bool foundInList = false;
        bool toDelete = false;
        ComponentInfo currentInfo = ComponentInfoFactory::fromListComponentsResponseMessage(msg);
        {
            ReaderGuard rg(this);
            for(ComponentInfo & myInfo: components)
            {

                if(myInfo.name == currentInfo.name)
                {
                    //LOG(Debug) << "My: " << myInfo.id << " " << myInfo.name << " Remote: " << currentInfo.id << " " << currentInfo.name << std::endl;
                    foundInList = true;
                    if(!msg->deleted)
                    {
                        //TODO implement comparison for component info
                        myInfo = currentInfo;
                        emit componentChanged(myInfo);

                    }
                    else
                    {

                        toDelete = true;
                        //TODO delete from list more efficient
                        LOG(Info) << "Deleting: " << myInfo.name << std::endl;
                        emit componentDeleted(myInfo);

                    }
                    break;
                }
            }
        }

        {

            if(!foundInList)
            {
                //Writing to Components
                std::unique_lock<std::mutex> lck(componentsMutex);
                while (componentsReader > 0)
                {
                    componentsCV.wait(lck);
                }
                components.push_back(currentInfo);

                lck.unlock(); //Connected functions could try to read -> deadlock!
                emit newComponentFound(currentInfo);
            }
            if(toDelete)
            {
                //Writing to Components
                std::unique_lock<std::mutex> lck(componentsMutex);
                while (componentsReader > 0)
                {
                    componentsCV.wait(lck);
                }

                size_t pos = 0;
                for(auto & info: components)
                {
                    if( info.id == msg->id)
                    {
                        break;
                    }
                    pos++;
                }
                components.erase(components.begin()+pos);
            }
        }
    }
}

void ComponentManager::generateResponse()
{
    auto responseFunc = [&]()
    {
        if(baseEntity)
        {
            ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::fromEntity(this->baseEntity).toRosMessage();
            msg->nodename = rosNode->get_name();
            msg->deleted = false;
            this->listComponentsResponsePublisher->publish(msg);
            std::function<void(EntityBase::SharedPtr)> iteratingFunc = [&](EntityBase::SharedPtr ent)
            {
                auto respMsg = ComponentInfoFactory::fromEntity(ent).toRosMessage();
                respMsg->nodename = rosNode->get_name();
                respMsg->deleted = false;
                this->listComponentsResponsePublisher->publish(respMsg);
            };

            this->baseEntity->iterateThroughAllChilds(iteratingFunc);
        }
    };

    responseFunc();
}

void ComponentManager::onChildAdded(EntityBase::SharedPtr child, EntityBase::SharedPtr parent, bool remote)
{
    UNUSED(parent);
    UNUSED(remote);
    LOG(Info) << "Child added: " << child->getName() << std::endl;
    //Connect to add events
    connect(child.get(),&EntityBase::childAdded,this, &ComponentManager::onChildAdded);
    connect(child.get(),&EntityBase::childRemoved,this, &ComponentManager::onChildRemoved);
    connect(child.get(), &EntityBase::entityDeleted,this, &ComponentManager::onEntityDeleted);
    //Publish component information
    ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::fromEntity(child).toRosMessage();
    msg->nodename = rosNode->get_name();
    msg->deleted = false;
    this->listComponentsResponsePublisher->publish(msg);

}

void ComponentManager::onChildRemoved(EntityBase::SharedPtr child, EntityBase::SharedPtr parent, bool remote)
{
    UNUSED(parent);
    UNUSED(remote);
    LOG(Info) << "Child removed: " << child->getName() << std::endl;
    disconnect(child.get(), &EntityBase::childAdded, this, &ComponentManager::onChildAdded);
    disconnect(child.get(), &EntityBase::childRemoved, this, &ComponentManager::onChildRemoved);

    ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = ComponentInfoFactory::fromEntity(child).toRosMessage();
    msg->nodename = rosNode->get_name();
    msg->deleted = true;
    this->listComponentsResponsePublisher->publish(msg);
}

void ComponentManager::onEntityDeleted(ComponentInfo info)
{


    LOG(Warning) << info.name << " got deleted" << std::endl;
    ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg = info.toRosMessage();
    msg->nodename = rosNode->get_name();
    msg->deleted = true;
    this->listComponentsResponsePublisher->publish(msg);
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
