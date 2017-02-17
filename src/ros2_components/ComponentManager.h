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

#pragma once
/*Qt*/
#include <QObject>

/*stdlib*/
#include <functional>
#include <cstdlib>
#include <iostream>
#include <ctime>

/*ROS2*/
#include "rclcpp/rclcpp.hpp"

/*Messages*/
#include "ros2_components_msg/msg/component_changed.hpp"
#include "ros2_components_msg/msg/list_components_request.hpp"
#include "ros2_components_msg/msg/list_components_response.hpp"

/*ros2_components*/
#include "ros2_simple_logger/Logger.h"

#include "AdvertisementType.h"
#include "EntityFactory.h"
#include "ComponentInfo.h"
#include "ComponentInfoFactory.h"
#include "ComponentListFilter.h"

namespace ros2_components
{
/**
 * @brief The ComponentManager class - Collects component/entity advertisements in the systems and represents them ordered to the user
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
     * Use this constructor if your having a node that doesnt need a base entity (for example an algorithm node)
     */
    ComponentManager(rclcpp::node::Node::SharedPtr _localNode);
    /**
     * @brief ComponentManager
     * @param _localNode
     * @param _baseEntity
     * Use this constructor if your having a node that needs a base entity (for example a hardware sensor/actor node)
     */
    ComponentManager(rclcpp::node::Node::SharedPtr _localNode, EntityBase::SharedPtr _baseEntity);
    /**
     * @brief IDAlreadyInUse
     * @param id
     * @return true if id is already in use
     *
     * WARNING: This function can't guarantee 100% that an id isn't used in the system. If an component is created but not published to the system this function will return true even if the id is in use.
     */
    bool IDAlreadyInUse(uint64_t id);
    /**
     * @brief ListComponents
     * @return vector of all currently listed components
     * TODO think of a better way to access all components
     */
    std::vector<ComponentInfo> ListComponents();
    /**
     * @brief CountComponents
     * @return
     */
    int64_t CountComponents();
    /**
     * @brief ListNodes
     * @return List of all found nodes
     */
    std::vector<std::string> ListNodes();
    /**
     * @brief ListComponentsBy
     * @param filter
     * @return List of ComponentInfo objects that are matched by the given ComponentListFilter
     */
    std::vector<ComponentInfo> ListComponentsBy(ComponentListFilter filter);
    /**
     * @brief GetInfoToId
     * @param id
     * @param success
     * @return ComponentInfo to the given id
     */
    ComponentInfo GetInfoToId(uint64_t id, bool* success = 0);

    /**
     * @brief UpdateComponentsList
     * Publishes a message to the ListComponentsRequest topic.
     */
    void UpdateComponentsList();
    /**
     *  Rebuild Component from the given id, pass true for rebuild Hierarchy to rebuild an entity tree (for example give the robot and and pass true in order to rebuild the whole component tree)
     */
    template<typename T>
    std::shared_ptr<T> RebuildComponent(int64_t id,bool rebuildHierarchy = false)
    {
        ComponentInfo relavantInfo;
        bool found = false;
        for(auto & info : Components)
        {
            if(info.id == id)
            {
                relavantInfo = info;
                found = true;
            }
        }
        if(!found)
            throw std::runtime_error("Could not find a component with the given id");
        std::shared_ptr<T> ent = RebuildComponent<T>(relavantInfo,rebuildHierarchy);
        return ent;
    }
    /**
     * Rebuild a component from a ComponentInfo object using the EntityFactory
     */
    template<typename T>
    std::shared_ptr<T> RebuildComponent(ComponentInfo & info,bool rebuildHierarchy = false)
    {
        if(!EntityFactory::Contains(info.type))
            throw std::runtime_error("Can't auto-rebuild this component: \" "+info.type +"\" - did register it to the EntityFactory");

        QGenericArgument subscribeArg;
        QGenericArgument idArg = Q_ARG(int64_t, info.id);

        //TODO - Check if this works
       if(!info.subscriber)
            subscribeArg = Q_ARG(bool, true);
        else
            subscribeArg = Q_ARG(bool, false);
        QGenericArgument nodeArg  =Q_ARG(std::shared_ptr< rclcpp::node::Node >, RosNode);

        std::shared_ptr<EntityBase> ent = EntityFactory::CreateInstanceFromName(info.type,idArg,subscribeArg,nodeArg);
        std::shared_ptr<T> secEnt = dynamic_pointer_cast<T>(ent);

        //The following line recusivly rebuild the tree structure that was published before
        if(rebuildHierarchy)
        {
            std::function<void(EntityBase::SharedPtr, ComponentInfo)> rec_build = [&](EntityBase::SharedPtr parentEntity,ComponentInfo parentInfo)
            {
                int a = 0;
                for(auto & child_id: parentInfo.childIds)
                {
                    bool success = false;
                    ComponentInfo childInfo = GetInfoToId(child_id,&success);
                    if(!success)
                        continue;
                    idArg = Q_ARG(int64_t, childInfo.id);

                    //LOG(Debug) << "Childinfo : subscriber: " << childInfo.subscriber << std::endl;
                    if(!childInfo.subscriber)
                        subscribeArg = Q_ARG(bool, true);
                    else
                        subscribeArg = Q_ARG(bool, false);
                    std::shared_ptr<EntityBase> child_obj = EntityFactory::CreateInstanceFromName(childInfo.type,idArg,subscribeArg,nodeArg);


                    parentEntity->addChild(child_obj);
                    rec_build(child_obj,childInfo);
                }
            };
            std::shared_ptr<EntityBase> parentEnt = dynamic_pointer_cast<EntityBase>(secEnt);
            rec_build(parentEnt,info);

            //TODO work on better paramter mechanism
           /* auto func = []( std::shared_ptr<EntityBase> ent){
                ent->updateParameters();
            };
            parentEnt->updateParameters();
            parentEnt->IterateThroughAllChilds(func);*/

        }

        return secEnt;

    }




private:
    /*Ros2 stuff*/
    /**
     * @brief RosNode
     */
    rclcpp::node::Node::SharedPtr RosNode;
    /**
     * @brief ComponentChangedSubscription
     * This topic keeps you informed about changes to components (entities) in the system - but not about new components
     */
    std::shared_ptr<rclcpp::subscription::Subscription<ros2_components_msg::msg::ComponentChanged>> ComponentChangedSubscription;
    /**
     * @brief ListComponentsRequestSubscription
     * On this topic a component manager can publish a request to all other component manager instances in the system in order to have them publish all their managed components to the
     * ListComponentResponse topic
     */
    rclcpp::subscription::Subscription<ros2_components_msg::msg::ListComponentsRequest>::SharedPtr ListComponentsRequestSubscription;
    /**
     * @brief ListComponentsResponseSubscription
     * On this topic the answer to the ListComponentsRequest is published
     */
    rclcpp::subscription::Subscription<ros2_components_msg::msg::ListComponentsResponse>::SharedPtr ListComponentsResponseSubscription;
    /**
     * @brief ComponentChangedPublisher
     * @see ComponentChangedSubscription
     */
    rclcpp::publisher::Publisher<ros2_components_msg::msg::ComponentChanged>::SharedPtr ComponentChangedPublisher;
    /**
     * @brief ListComponentsRequestPublisher
     * @see ListComponentsRequestSubscription
     */
    rclcpp::publisher::Publisher<ros2_components_msg::msg::ListComponentsRequest>::SharedPtr ListComponentsRequestPublisher;
    /**
     * @brief ListComponentsResponsePublisher
     * @see ListComponentsResponseSubscription
     */
    rclcpp::publisher::Publisher<ros2_components_msg::msg::ListComponentsResponse>::SharedPtr ListComponentsResponsePublisher;
    /**
     * @brief ComponentChangedCallback
     * @param msg
     * Gets called if a new message on the ComponentChanged topic is available
     */
    void ComponentChangedCallback(ros2_components_msg::msg::ComponentChanged::SharedPtr msg);
    /**
     * @brief ListComponentsRequestCallback
     * @param msg
     * Gets called if a new message on the ListComponentsRequest topic is available
     */
    void ListComponentsRequestCallback(ros2_components_msg::msg::ListComponentsRequest::SharedPtr msg);
    /**
     * @brief ListComponentsResponseCallback
     * @param msg
     * Gets called if a new message on the ListComponentsResponse topic is available
     */
    void ListComponentsResponseCallback(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg);

    void ProcessNewAdvertisment(const ros2_components_msg::msg::ComponentChanged::SharedPtr msg, ComponentInfo info);
    void ProcessChangeAdvertisment(const ros2_components_msg::msg::ComponentChanged::SharedPtr msg,ComponentInfo info);
    void ProcessDeleteAdvertisment(const ros2_components_msg::msg::ComponentChanged::SharedPtr msg,ComponentInfo info);

    /**
     * @brief Components
     * Stored components
     */
    std::vector<ComponentInfo> Components;
    EntityBase::SharedPtr BaseEntity;
    rmw_qos_profile_t component_manager_profile;
signals:
    //Qt signals that can be used in order to stay informed about changes in the system
    void NewComponentFound(ComponentInfo &info);
    void ComponentDeleted(ComponentInfo &info);
    void ComponentChanged(ComponentInfo &info);

};
}


