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
    Q_INVOKABLE Robot( int64_t _id, bool _subscribe ,std::shared_ptr<rclcpp::node::Node> parentNode, std::string  _name);

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

    template<typename T>
    static std::shared_ptr<T> RebuildRobotFromId(int64_t id, std::shared_ptr<rclcpp::node::Node> _parentNode )
    {
        LOG(LogLevel::Info) << "Building Robot from id: " << id <<std::endl;


        auto reqFunc = [&] (int64_t _id, std::shared_ptr<rclcpp::node::Node> _node, std::string basename) -> ros2_components_msg::srv::ListChilds::Response::SharedPtr
        {
            std::string name = basename+std::to_string(_id)+"_srv";
            LOG(LogLevel::Debug) << "Looking for service with name: " << name << std::endl;
            auto client = _node->create_client<ros2_components_msg::srv::ListChilds>(name);

            auto request = std::make_shared<ros2_components_msg::srv::ListChilds::Request>();
            auto result = client->async_send_request(request);

            // Wait for the result.

            if (result.wait_for(15_s) == std::future_status::ready)
            {
                return result.get();
            }
            else
                throw std::runtime_error("Could not contact robot");


        };


        QGenericArgument idArg = Q_ARG(int64_t, id);
        QGenericArgument subscribeArg = Q_ARG(bool, true);
        QGenericArgument nodeArg  =Q_ARG(std::shared_ptr< rclcpp::node::Node >, _parentNode);

        std::shared_ptr<T> robot =std::make_shared<T>(id,true,_parentNode);
        EntityBase::SharedPtr robotAsBaseEntity = dynamic_pointer_cast<T>(robot);
        //dynamic_pointer_cast<T>(EntityFactory::CreateInstanceFromName(robotType, idArg, subscribeArg, nodeArg));

        std::function<void(std::shared_ptr<ros2_components_msg::srv::ListChilds::Response> ,EntityBase::SharedPtr )> rec_req = [&](std::shared_ptr<ros2_components_msg::srv::ListChilds::Response> response,EntityBase::SharedPtr parent)
        {
            for(uint64_t i = 0; i< response->childids.size(); i++)
            {

                int64_t compId = response->childids[i];
                idArg = Q_ARG(int64_t, compId);
                LOG(LogLevel::Debug) <<response->childtypes[i]<< std::endl;
                if(response->childtypes[i].find("Actor") != std::string::npos)
                {
                    LOG(LogLevel::Debug) << "Created  actor with id:" << compId << std::endl;
                    subscribeArg = Q_ARG(bool, false);

                }
                else
                {
                    LOG(LogLevel::Debug) << "Creating sensor with id:" << compId << std::endl;
                    subscribeArg = Q_ARG(bool, true);
                }

                EntityBase::SharedPtr child = EntityFactory::CreateInstanceFromName(response->childtypes[i], idArg,subscribeArg,nodeArg);

                parent->addChild(child);
                auto nextRes = reqFunc(compId, _parentNode, response->childtypes[i]);
                rec_req(nextRes, child);
            }

        };
        auto res = reqFunc(id, _parentNode, robotAsBaseEntity->getClassName());
        rec_req(res,robot);


        auto func = [&](EntityBase::SharedPtr base)
        {
            base->updateParameters();
        };
        robot->IterateThroughAllChilds(func);
        LOG(LogLevel::Info) << "Creation of robot finished: "<<robot->getName() << std::endl;
        return robot;
    }

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
        if(parentComp == NULL)
        {
            LOG(Warning) << "Could not find parent component to new reported component: " << componentId << " : " << componentType << " : " << "Parent should have id: " << parentId << std::endl;
            LOG(Fatal) << "This could result into a not correctly rebuild robot model" << std::endl;
            return;
        }
        if(added)
        {
            QGenericArgument idArg = Q_ARG(int64_t, componentId);
            QGenericArgument subscribeArg = Q_ARG(bool, true);
            QGenericArgument nodeArg  =Q_ARG(std::shared_ptr< rclcpp::node::Node >, parentNode);

            EntityBase::SharedPtr comp = EntityFactory::CreateInstanceFromName(componentType, idArg, subscribeArg, nodeArg);
            if(comp == NULL)
                throw std::runtime_error("Could not rebuild new component");

            parentComp->addChild(comp, true);
           
            emit remote_entity_added(comp);
        }
        else
        {
            parentComp->removeChild(localComp);
            emit remote_entity_removed(localComp);
        }
    }

protected slots:
    virtual void on_child_added(std::shared_ptr<EntityBase> child,std::shared_ptr<EntityBase> parent, int depth, bool remote);
    virtual void on_child_removed(std::shared_ptr<EntityBase> child,std::shared_ptr<EntityBase> parent, int depth, bool remote);
private:
    std::mutex childAdded_mutex;

signals:
    void remote_entity_added(std::shared_ptr<EntityBase> child);
    void remote_entity_removed(std::shared_ptr<EntityBase> child);





};

}

