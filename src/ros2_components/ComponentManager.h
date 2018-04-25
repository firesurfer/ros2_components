/*
 * Copyright 2018 <Lennart Nachtigall> <firesurfer127@gmail.com>
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

#pragma once
/*Qt*/
#include <QObject>

/*stdlib*/
#include <functional>
#include <cstdlib>
#include <iostream>
#include <ctime>
#include <list>
#include <memory>
#include <chrono>
#include <mutex>
#include <condition_variable>

/*ROS2*/
#include "rclcpp/rclcpp.hpp"

/*Messages*/
#include "ros2_components_msg/msg/component_changed.hpp"
#include "ros2_components_msg/msg/list_components_request.hpp"
#include "ros2_components_msg/msg/list_components_response.hpp"

/*ros2_components*/
#include "ros2_simple_logger/Logger.h"
#include "ros2_components_exceptions.h"
#include "NodeContainer.h"

#include "EntityFactory.h"
#include "ComponentInfo.h"
#include "ComponentInfoFactory.h"

namespace ros2_components
{
/**
 * @brief The ComponentManager class - Collects component/entity advertisements in the systems and represents them ordered to the user.
 */
class ComponentManager:public QObject
{
    Q_OBJECT
public:
    //Typedef for easy to use SharedPtr
    typedef std::shared_ptr<ComponentManager> SharedPtr;
    /**
     * @brief ComponentManager
     * @param _localNode
     */
    ComponentManager(NodeContainer::SharedPtr _nodeContainer);
    /**
     * @brief ~ComponentManager - destructor. Waits for responder thread
     */
    virtual ~ComponentManager();
    /**
     * Tell the component manager to start publishing its entities. This should be called once the base entity is ready
     * and all its childs are added to it (usually at the end of setup()). After calling this, dynamically added components will be published automatically.
     * If the node does not have components it wants to publish, this does not have to be called
     * @param _baseEntity the nodes' base entity
     */
    void registerComponents(EntityBase::SharedPtr _baseEntity);

    /**
     * Enables component listing of other nodes.
     **/
    void enableComponentHandling();

    /**
     * @brief listComponents
     * @return vector of all currently listed components
     */

    std::vector<ComponentInfo> listComponents();
    /**
     * @brief listNodes
     * @return List of all found nodes
     */
    std::vector<ComponentInfo> listNodes();
    /**
     * @brief listComponentsBy
     * @param filter
     * @return List of ComponentInfo objects that are matched by the given ComponentListFilter
     */
    std::vector<ComponentInfo> listComponentsBy(std::function<bool(const ComponentInfo&)> filter);
    /**
     * @brief getInfoWithFilter gets the first Component which matches the filter
     * @param filter the filter
     * @param success will be set to true if a Component was found before timeout and false if not
     * @param timeout in milliseconds. Waits indefinitely if negative
     * @throws AlreadySpinningException if a rclcpp::executor is already running, and this method waits for the Component. Will not happen with a timeout of 0ms
     * @throws TimeoutException when no component matching the filter was found after {@param timeout} time
     */
    ComponentInfo getInfoWithFilter(std::function<bool(const ComponentInfo&)> filter, bool* success = nullptr, std::chrono::milliseconds timeout = std::chrono::milliseconds::zero());
    /**
     * @brief getInfoToId
     * @param id
     * @param success will be set to true if a Component was found before timeout and false if not
     * @param timeout in milliseconds. Waits indefinitely if negative
     * @throws AlreadySpinningException if a rclcpp::executor is already running, and this method waits for the Component. Will not happen with a timeout of 0ms
     * @throws TimeoutException when no component with id {@param id} was found after {@param timeout} time
     */
    ComponentInfo getInfoToId(int64_t id, bool* success = nullptr, std::chrono::milliseconds timeout = std::chrono::milliseconds::zero());
    /**
     * @brief getInfoWithFilterAsync gets the first Component which matches the filter and calls the callback on it
     * @param callback the callback to call when the component is found
     * @param filter the filter
     * @param timeout will call @callback with an default-initialized ComponentInfo when it timeouts. Will not time out if @param timeout negative (default)
     */
    void getInfoWithFilterAsync(std::function<void(ComponentInfo)> callback, std::function<bool(const ComponentInfo&)> filter, std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));
    /**
     * @brief getInfoWithFilterAsync gets the first Component with given id and calls the callback on it
     * @param callback the callback to call when the component is found
     * @param id the id
     * @param timeout will call @callback with an default-initialized ComponentInfo when it timeouts. Will not time out if @param timeout negative (default)
     */
    void getInfoToIdAsync(std::function<void(ComponentInfo)> callback, int64_t id, std::chrono::milliseconds timeout = std::chrono::milliseconds(-1));
    /**
     * @brief updateComponentsList
     * Publishes a message to the listComponentsRequest topic.
     */
    void updateComponentsList();

    /**
     *  Rebuild first Component which matches the given filter, pass true for rebuild Hierarchy to rebuild an entity tree (for example give the robot and and pass true in order to rebuild the whole component tree)
     *  @param rebuildHierarchy, wait for and rebuild all childs as well, defaults to false
     *  @param timeout in milliseconds, throws exception if no component is found before timeout. Waits indefinitely if negative; defaults to 0ms
     *  @throws TimeoutException when no component matching the filter was found after {@param timeout} time and/ or not all its children were found if {@param rebuildHierarchy} is true
     * @throws AlreadySpinningException if a rclcpp::executor is already running, and this method waits for the Component. Will not happen with a timeout of 0ms
     *  @throws EntityCastException when the filter-matching component can not be cast to type T
     */
    template<typename T>
    std::shared_ptr<T> rebuildComponent(std::function<bool(const ComponentInfo&)> filter, bool rebuildHierarchy = false, std::chrono::milliseconds timeout = std::chrono::milliseconds::zero())
    {
        auto startTime = std::chrono::system_clock::now();
        bool waitIndefinitely = timeout < std::chrono::milliseconds::zero();
        bool found;
        ComponentInfo relevantInfo = getInfoWithFilter(filter, &found, timeout);

        if (found)
        {
            std::shared_ptr<T> ent;
            if (waitIndefinitely)
            {
                ent = rebuildComponent<T>(relevantInfo, rebuildHierarchy, timeout);
            }
            else
            {
                auto remaining_timeout = startTime + timeout - std::chrono::system_clock::now();
                if (remaining_timeout < std::chrono::milliseconds::zero())
                {
                    remaining_timeout = std::chrono::milliseconds::zero();
                }
                ent = rebuildComponent<T>(relevantInfo, rebuildHierarchy, std::chrono::duration_cast<std::chrono::milliseconds>(remaining_timeout));
            }
            return ent;
        }
        else
        {
            throw TimeoutException();
        }
    }
    /**
     *  Rebuild Component from the given id, pass true for rebuild Hierarchy to rebuild an entity tree (for example give the robot and and pass true in order to rebuild the whole component tree)
     *  @param rebuildHierarchy, wait for and rebuild all childs as well, defaults to false
     *  @param timeout in milliseconds, throws exception if no component is found before timeout. Waits indefinitely if negative; defaults to 0ms
     *  @throws TimeoutException when no component with id {@param id} was found after {@param timeout} time and/ or not all its children were found if {@param rebuildHierarchy} is true
     * @throws AlreadySpinningException if a rclcpp::executor is already running, and this method waits for the Component. Will not happen with a timeout of 0ms
     *  @throws EntityCastException when the filter-matching component can not be cast to type T
     */
    template<typename T>
    std::shared_ptr<T> rebuildComponent(int64_t id, bool rebuildHierarchy = false, std::chrono::milliseconds timeout = std::chrono::milliseconds::zero())
    {
        auto filter = [id] (const ComponentInfo& info) -> bool
        {
            return (info.id == id);
        };
        return rebuildComponent<T>(filter, rebuildHierarchy, timeout);
    }

    /**
     *  Rebuild first Component which matches the given filter and call the callback with it, will return immediately
     *  @param callback the callback expecting the rebuild component or an empty ptr if it timed out
     *  @param filter the filter
     *  @param rebuildHierarchy, wait for and rebuild all childs as well, defaults to false
     *  @param timeout in milliseconds, will pass empty shared_ptr to the callback if it times out. Waits indefinitely if negative; defaults to negative
     */
    template<typename T>
    void rebuildComponentAsync(std::function<void(std::shared_ptr<T>)> callback, std::function<bool(const ComponentInfo&)> filter, bool rebuildHierarchy = false, std::chrono::milliseconds timeout = std::chrono::milliseconds(-1))
    {
        //Nested shared_ptr is intended here
        std::shared_ptr<std::shared_ptr<T> > entityCache = std::make_shared<std::shared_ptr<T> >(nullptr);
        std::shared_ptr<ComponentInfo> parent = std::make_shared<ComponentInfo>();
        std::shared_ptr<std::list<int64_t> > relevantIds = std::make_shared<std::list<int64_t> >();
        auto fullFilter = [filter, rebuildHierarchy, entityCache, parent, relevantIds, this] (const ComponentInfo& info) -> bool
        {
            bool relevant = false;

            if (filter(info) && parent->name.empty())
            {
                relevant = true;
                *parent = info;
                for (auto childId : info.childIds)
                {
                    relevantIds->push_back(childId);
                }
            }
            for (auto it = relevantIds->begin(); it != relevantIds->end(); it++)
            {
                if (*it == info.id)
                {
                    relevant = true;
                    relevantIds->erase(it);
                    for (auto childId : info.childIds)
                    {
                        relevantIds->push_back(childId);
                    }
                    break;
                }
            }

            if (relevant && relevantIds->empty())
            {
                *entityCache = rebuildComponent<T>(*parent, rebuildHierarchy);
                return true;
            }
            return false;
        };

        auto fullCallback = [callback, entityCache] (ComponentInfo info)
        {
            if (info.name.empty())
            {
                //Timeout
                callback(std::shared_ptr<T>());
            }
            else
            {
                callback(*entityCache);
            }
        };

        getInfoWithFilterAsync(fullCallback, fullFilter, timeout);
    }
    /**
     *  Rebuild Component from the given id and call the callback with it, will return immediately
     *  @param callback the callback expecting the rebuild component or an empty ptr if it timed out
     *  @param id the id of the component
     *  @param rebuildHierarchy, wait for and rebuild all childs as well, defaults to false
     *  @param timeout in milliseconds, will pass empty shared_ptr to the callback if it times out. Waits indefinitely if negative (default)
     */
    template<typename T>
    void rebuildComponentAsync(std::function<void(std::shared_ptr<T>)> callback, int64_t id, bool rebuildHierarchy = false, std::chrono::milliseconds timeout = std::chrono::milliseconds(-1))
    {
        auto filter = [id] (const ComponentInfo& info) -> bool
        {
            return (info.id == id);
        };
        rebuildComponentAsync(callback, filter, rebuildHierarchy, timeout);
    }

    /**
     * Rebuild a component from a ComponentInfo object using the EntityFactory
     * @throws EntityCastException when the filter-matching component can not be cast to type T
     */
    template<typename T>
    std::shared_ptr<T> rebuildComponent(ComponentInfo & info,bool rebuildHierarchy = false, std::chrono::milliseconds timeout = std::chrono::milliseconds::zero())
    {
        std::shared_ptr<T> entity = dynamic_pointer_cast<T>(rebuildComponent(info, rebuildHierarchy, false, timeout));
        if(!entity)
            throw EntityCastException();
        return entity;
    }
    /**
     * rebuildComponent helper method
     */
    std::shared_ptr<EntityBase> rebuildComponent(const ComponentInfo & info, bool rebuildHierarchy = false, bool forcePubOrSubChange = false, std::chrono::milliseconds timeout = std::chrono::milliseconds::zero());

    /**
     * @brief enableComponentTimeout
     * Enables a timer that checks if we haven't heard from a components for 60 seconds,
     * then aks if the component is still there and if there havn't been an answer in 60 seconds it deletes the components.
     */
    void enableComponentTimeout();

private:
    std::mutex componentsMutex;
    std::vector<ComponentInfo> components;
    NodeContainer::SharedPtr nodeContainer;
    std::mutex callbacksMutex;
    std::list<QMetaObject::Connection> callbacks;
    uint64_t componentsReader;
    std::condition_variable componentsCV;
    rmw_qos_profile_t component_manager_profile;
    void generateResponse();

    bool handle_components = false;

    /*Needed stuff for components timeout*/
    bool enable_components_timeout = false;
    rclcpp::TimerBase::SharedPtr components_timeout_timer;
    std::map<int64_t, std::chrono::steady_clock::time_point> components_times;
    std::map<int64_t, std::chrono::steady_clock::time_point> components_last_request_times;
    //The time in seconds we will ask if a component is still there and if it doesn't answer in the
    //same amount of time we remove it from the components list
    const std::chrono::seconds components_timeout_garbage_collect_time = std::chrono::seconds(60);

    /*Ros2 stuff*/
    /**
     * @brief rosNode
     */
    rclcpp::Node::SharedPtr rosNode;
    EntityBase::SharedPtr baseEntity;
    /**
     * @brief componentChangedSubscription
     * This topic keeps you informed about changes to components (entities) in the system - but not about new components
     */
    std::shared_ptr<rclcpp::Subscription<ros2_components_msg::msg::ComponentChanged>> componentChangedSubscription;
    /**
     * @brief listComponentsRequestSubscription
     * On this topic a component manager can publish a request to all other component manager instances in the system in order to have them publish all their managed components to the
     * ListComponentResponse topic
     */
    rclcpp::Subscription<ros2_components_msg::msg::ListComponentsRequest>::SharedPtr listComponentsRequestSubscription;
    /**
     * @brief listComponentsResponseSubscription
     * On this topic the answer to the listComponentsRequest is published
     */
    rclcpp::Subscription<ros2_components_msg::msg::ListComponentsResponse>::SharedPtr listComponentsResponseSubscription;

    /**
     * @brief listComponentsRequestPublisher
     * @see listComponentsRequestSubscription
     */
    rclcpp::Publisher<ros2_components_msg::msg::ListComponentsRequest>::SharedPtr listComponentsRequestPublisher;
    /**
     * @brief listComponentsResponsePublisher
     * @see listComponentsResponseSubscription
     */
    rclcpp::Publisher<ros2_components_msg::msg::ListComponentsResponse>::SharedPtr listComponentsResponsePublisher;

    /**
     * @brief listComponentsRequestCallback
     * @param msg
     * Gets called if a new message on the listComponentsRequest topic is available
     */
    void listComponentsRequestCallback(ros2_components_msg::msg::ListComponentsRequest::SharedPtr msg);
    /**
     * @brief listComponentsResponseCallback
     * @param msg
     * Gets called if a new message on the listComponentsResponse topic is available
     */
    void listComponentsResponseCallback(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg);

    /**
     * @brief collect_timed_out_components
     */
    void collect_timed_out_components();


    //RAII helper to manage reader entrance on Components
    class ReaderGuard
    {
    public:
        ReaderGuard(ComponentManager*);
        ~ReaderGuard();

    private:
        ComponentManager* cm;
    };

signals:
    //Qt signals that can be used in order to stay informed about changes in the system
    void newComponentFound(ComponentInfo info);
    void componentDeleted(ComponentInfo info);
    void componentChanged(ComponentInfo info);
private slots:
    void onChildAdded(EntityBase::SharedPtr child, EntityBase::SharedPtr parent, bool remote);
    void onChildRemoved(EntityBase::SharedPtr child, EntityBase::SharedPtr parent, bool remote);
    void onEntityDeleted(ComponentInfo info);
};
}


