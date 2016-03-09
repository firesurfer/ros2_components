/*
 * Copyright 2016 <copyright holder> <email>
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
#include "ros2_components/Entity.h"
#include "std_msgs/msg/empty.hpp"
#include "ros2_components_msg/msg/new_component_added.hpp"
#include "ros2_components/EntityFactory.h"
#include "ros2_simple_logger/ConsoleColor.h"
#include <mutex>

namespace ros2_components
{

class Robot :  public Entity< ros2_components_msg::msg::NewComponentAdded>
{
    Q_OBJECT
public:
    typedef  std::shared_ptr<Robot> SharedPtr ;
    Robot( int64_t _id, bool _subscribe ,std::shared_ptr<rclcpp::node::Node> parentNode, std::string  _name);

    template<typename T>
    std::vector<std::shared_ptr<T>> GetAllChildsOfType()
    {
        std::vector<std::shared_ptr<T>> elements;
        std::function<void(std::shared_ptr<EntityBase>)> GoThrough = [&] (std::shared_ptr<EntityBase> _parent)
        {
            for(auto & child : _parent->getAllChilds())
            {
                std::shared_ptr<T> casted_child = dynamic_pointer_cast<T>(child);
                if(casted_child != NULL)
                {
                    elements.push_back(casted_child);
                }
                GoThrough(child);
            }
        };
        return elements;
    }

    void IterateThroughAllChilds(std::function<void(EntityBase::SharedPtr)> func)
    {
        std::function<void(EntityBase::SharedPtr)> rec_func = [&](EntityBase::SharedPtr base){
            for(auto &  child : base->getAllChilds())
            {
                func(child);
                rec_func(child);

            }

        };
        for(auto & seg: getAllChilds())
        {
            func(seg);
            rec_func(seg);
        }
    }
    template<typename T>
    void IterateThroughAllChildsOfType(std::function<void(std::shared_ptr<T>)> func)
    {
        auto callbackFunc = [&](EntityBase::SharedPtr item)
        {
            std::shared_ptr<T> ch = dynamic_pointer_cast<T>(item);
            if(ch != NULL)
                func(ch);

        };
        IterateThroughAllChilds(callbackFunc);
    }

    void virtual  PrintTree();

    static std::vector<int64_t> ListKnownRobots(std::shared_ptr<rclcpp::node::Node> _parentNode,std::string prefix = "");

signals:
    void hardwareConnection(EntityBase::SharedPtr seg);
protected:

    virtual void listenerCallback(const ros2_components_msg::msg::NewComponentAdded::SharedPtr  msg)
    {
        int64_t parentId = msg->parentid;
        int64_t componentId = msg->componentid;
        std::string componentType = msg->componenttype;
        bool added = msg->added;

        EntityBase::SharedPtr parentComp;
        EntityBase::SharedPtr localComp;
        auto func = [&](EntityBase::SharedPtr child)
        {
            if(child->getId()== parentId)
            {
                parentComp = child;
                return;
            }
            if(!added && child->getId() == componentId)
            {
                localComp = child;
            }
        };
        IterateThroughAllChilds(func);
        if(added)
        {
            QGenericArgument idArg = Q_ARG(int64_t, componentId);
            QGenericArgument subscribeArg = Q_ARG(bool, true);
            QGenericArgument nodeArg  =Q_ARG(std::shared_ptr< rclcpp::node::Node >, parentNode);

            EntityBase::SharedPtr comp = EntityFactory::CreateInstanceFromName(componentType, idArg, subscribeArg, nodeArg);
            parentComp->addChild(comp);
            emit remote_entity_added(comp);
        }
        else
        {
            parentComp->removeChild(localComp);
            emit remote_entity_removed(localComp);
        }
    }

protected slots:
    virtual void on_child_added(std::shared_ptr<EntityBase> child,std::shared_ptr<EntityBase> parent, int depth);
    virtual void on_child_removed(std::shared_ptr<EntityBase> child,std::shared_ptr<EntityBase> parent, int depth);
private:
     std::mutex childAdded_mutex;
signals:
    void remote_entity_added(std::shared_ptr<EntityBase> child);
    void remote_entity_removed(std::shared_ptr<EntityBase> child);





};

}

