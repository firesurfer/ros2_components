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

#ifndef COMPONENTMANAGER_H
#define COMPONENTMANAGER_H

#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include "ros2_components_msg/msg/entity_advertisement.hpp"
#include "ComponentInfo.h"
#include "ros2_components/AdvertisementType.h"
#include "ros2_simple_logger/Logger.h"
#include "ros2_components/EntityFactory.h"
#include <functional>

#include <cstdlib>
#include <iostream>
#include <ctime>
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
     * @param _localNode - The ros node that should be used
     */
    ComponentManager(rclcpp::node::Node::SharedPtr _localNode, EntityBase::SharedPtr _baseEntity);
    /**
     * @brief IDAlreadyInUse
     * @param id
     * @return true if id is already in use
     *
     * WARNING: This function can't guarantee 100% that an id isn't used in the system. If an component is created but not published to the system this function will return true even if the id is in use.
     */
    bool IDAlreadyInUse(uint64_t id)
    {
        for(auto & myInfo: Components)
        {
            if(myInfo.id == id)
                return true;
        }
        return false;
    }


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
        return RebuildComponent<T>(relavantInfo,rebuildHierarchy);
    }
    /**
     * Rebuild a component from a ComponentInfo object using the EntityFactory
     */
    template<typename T>
    std::shared_ptr<T> RebuildComponent(ComponentInfo & info,bool rebuildHierarchy = false)
    {
        if(!EntityFactory::Contains(info.type))
            throw std::runtime_error("Can't auto-rebuild this component - did register it to the EntityFactory");

        QGenericArgument subscribeArg;
        QGenericArgument idArg = Q_ARG(int64_t, info.id);
        //Determine whether it's a sensor or an actor
        if(info.name.find("Sensor") != std::string::npos)
            subscribeArg = Q_ARG(bool, true);
        else
            subscribeArg = Q_ARG(bool, false);
        QGenericArgument nodeArg  =Q_ARG(std::shared_ptr< rclcpp::node::Node >, localNode);



        QObject * obj = EntityFactory::CreateInstanceFromName(info.type,idArg,subscribeArg,nodeArg);
        T* ent = dynamic_cast<T*>(obj);
        if(ent == NULL)
        {
            //Clean up
            delete obj;
            throw std::runtime_error("Could cast created object to the given type - WTF?!");
        }
        std::shared_ptr<T> secEnt(ent);

        //The following line recusivly rebuild the tree structure that was published before
        if(rebuildHierarchy)
        {
            std::function<void(EntityBase::SharedPtr, ComponentInfo)> rec_build = [&](EntityBase::SharedPtr parentEntity,ComponentInfo& parentInfo)
            {
                for(auto & child_id: parentInfo.childIds)
                {
                    bool success = false;
                    ComponentInfo childInfo = GetInfoToId(child_id,&success);
                    if(!success)
                        continue;
                    idArg = Q_ARG(int64_t, childInfo.id);
                    if(childInfo.name.find("Sensor") != std::string::npos)
                        subscribeArg = Q_ARG(bool, true);
                    else
                        subscribeArg = Q_ARG(bool, false);
                    QObject * child_obj = EntityFactory::CreateInstanceFromName(childInfo.type,idArg,subscribeArg,nodeArg);
                    EntityBase* child_ent = dynamic_cast<EntityBase*>(child_obj);
                    if(child_ent == NULL)
                    {
                        delete child_obj;
                        throw std::runtime_error("Could not cast created child object to EntityBase - This is a fatal error");
                    }
                    std::shared_ptr<EntityBase> sh_child_ent(child_ent);
                    parentEntity->addChild(sh_child_ent);
                    rec_build(sh_child_ent,childInfo);
                }
            };
            std::shared_ptr<EntityBase> parentEnt = dynamic_pointer_cast<EntityBase>(secEnt);
            rec_build(parentEnt,info);


            auto func = []( std::shared_ptr<EntityBase> ent){
                ent->updateParameters();
            };
            parentEnt->updateParameters();
            parentEnt->IterateThroughAllChilds(func);

        }

        return secEnt;

    }
    /**
     * @brief ListComponents
     * @return vector of all currently listed components
     */
    std::vector<ComponentInfo> ListComponents()
    {
        return Components;
    }
    /**
     * @brief GetInfoToId
     * @param id
     * @param success
     * @return ComponentInfo to the given id
     */
    ComponentInfo GetInfoToId(uint64_t id, bool* success = 0)
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

    //TODO create function that help sorting the component infos

private:
    /*Ros2 stuff*/
    rclcpp::node::Node::SharedPtr localNode;
    std::shared_ptr<rclcpp::subscription::Subscription<ros2_components_msg::msg::EntityAdvertisement>> AdvertisementSubscription;

    void AdvertisementCallback(ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg);
    /**
     * @brief Components
     * Stored components
     */
    std::vector<ComponentInfo> Components;
    void ProcessNewAdvertisment(const ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg,ComponentInfo info);
    void ProcessChangeAdvertisment(const ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg,ComponentInfo info);
    void ProcessDeleteAdvertisment(const ros2_components_msg::msg::EntityAdvertisement::SharedPtr msg,ComponentInfo info);
signals:
    //Qt signals that can be used in order to stay informed about changes in the system
    void NewComponentAvailable(ComponentInfo &info);
    void ComponentDeleted(ComponentInfo &info);
    void ComponentChanged(ComponentInfo &info);

};
}

#endif // COMPONENTMANAGER_H
