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
#include "EntityBase.h"
#include "EntityContainer.h"
#include <std_msgs/msg/empty.hpp>
namespace ros2_components {


template <typename MessageType>
class Entity : public EntityBase
{

public:
    typedef MessageType RosMessageType;
    /**
     * @brief Constructor of Entity
     * @param className is used together with the id to itentify topics, etc. of this entity
     */
    Entity(int64_t _id, bool _subscribe, NodeContainer::SharedPtr _nodeContainer, std::string _className) : EntityBase(_id, _subscribe, _nodeContainer, _className)
    {
        //Some ROS2 QOS Configuration -> Taken from an example
        custom_qos_profile = rmw_qos_profile_parameters;
        if(!isSubscriber())
        {
            entityPublisher = nodeContainer->create_publisher<MessageType>(getName(), custom_qos_profile);
            pubBase = entityPublisher;
        }
        else
        {
            using namespace std::placeholders;
            entitySubscription = nodeContainer->create_subscription<MessageType>(getName(), std::bind(&Entity::internalListenerCallback, this,_1), custom_qos_profile);
            subBase = entitySubscription;
        }
        LOG(LogLevel::Info) << "Created: " << getName() << " As a subscriber?: " << std::to_string(isSubscriber())<<std::endl;
    }

    //TODO deduplicate
    Entity(int64_t _id, bool _subscribe, NodeContainer::SharedPtr _nodeContainer, std::string _className, std::string _componentName):EntityBase(_id,_subscribe,_nodeContainer,_className,_componentName)
    {
        //Some ROS2 QOS Configuration -> Taken from an example
        custom_qos_profile = rmw_qos_profile_sensor_data;
        if(!isSubscriber())
        {
            entityPublisher = nodeContainer->create_publisher<MessageType>(getName(), custom_qos_profile);
            pubBase = entityPublisher;
        }
        else
        {
            using namespace std::placeholders;
            entitySubscription = nodeContainer->create_subscription<MessageType>(getName(), std::bind(&Entity::internalListenerCallback, this,_1), custom_qos_profile);
            subBase = entitySubscription;
        }
        LOG(LogLevel::Info) << "Created: " << getName() << " As a subscriber?: " << std::to_string(isSubscriber())<<std::endl;
    }


    virtual ~Entity() {

    }

    /**
     * @brief publish - Tell class to publish the current data to the world
     * @return
     */
    virtual bool publish()
    {
        LOG(Error) << "Entity : Please override publish function" << std::endl;
        return true;
    }


    /**
     * @brief add a new listener to be called when new data arrives
     */
    /*void addListener(std::function<void(typename MessageType::SharedPtr)> listener)
    {
        listeners.push_back(listener);
    }*/
    void addListener(std::function<void(EntityContainer)> listener)
    {
        //Think about entity wrapper class that allows casts
        ptrListeners.push_back(listener);

    }



protected:
    /**
     * @brief This is the method to handle new data inside your entity
     */
    //TODO make this method abstract in order to force implementation in subclasses
    virtual void listenerCallback(const typename MessageType::SharedPtr  msg)
    {
        LOG(Error) << "Please override listenerCallback" << std::endl;
        //To ignore warning
        UNUSED(msg);

    }

    bool publishMsg(std::shared_ptr<MessageType> msg)
    {
        if(!this->isSubscriber() && this->entityPublisher)
        {
            entityPublisher->publish(msg);
            return true;
        }
        return false;
    }

    //ROS 2 Stuff
    rmw_qos_profile_t custom_qos_profile;

    std::shared_ptr<rclcpp::Publisher<MessageType>> entityPublisher;
    std::shared_ptr<rclcpp::Subscription<MessageType>> entitySubscription;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterEventSubscription;




private:
    std::vector<std::function<void(typename MessageType::SharedPtr)>> listeners;
    std::vector<std::function<void(EntityContainer)>> ptrListeners;
    /**
     * @brief calls the rest of the registerd listeners
     */
    void internalListenerCallback(const typename MessageType::SharedPtr msg)
    {
        if(msg)
        {
            listenerCallback(msg);
            emit newData();
            for(auto listener : listeners) {
                if(listener)
                    listener(msg);
            }
            for(auto listener: ptrListeners)
            {
                if(listener)
                {
                    std::shared_ptr<EntityBase> ent = dynamic_pointer_cast<EntityBase>(shared_from_this());
                    if(ent)
                        listener(EntityContainer(ent));
                }
            }
        }
    }

};

//Specialisation for empty messages -> don't create publisher and subscription
template<>
class Entity<std_msgs::msg::Empty> :public EntityBase
{
public:
    Entity(int64_t _id, bool _subscribe, NodeContainer::SharedPtr _nodeContainer, std::string _className) : EntityBase(_id, _subscribe, _nodeContainer, _className)
    {

    }

    Entity(int64_t _id, bool _subscribe,NodeContainer::SharedPtr _nodeContainer, std::string _className, std::string _componentName):EntityBase(_id,_subscribe,_nodeContainer,_className,_componentName)
    {

    }

    /**
     * @brief publish - Tell class to publish the current data to the world
     * @return
     */
    virtual bool publish()
    {
        LOG(Error) << "Entity : Please override publish function" << std::endl;
        return true;
    }


    void addListener(std::function<void(EntityContainer)> listener)
    {
        UNUSED(listener);
    }
private:
    void internalListenerCallback(const typename std_msgs::msg::Empty::SharedPtr msg)
    {
        UNUSED(msg);
    }
protected:
    virtual void listenerCallback(const typename std_msgs::msg::Empty::SharedPtr  msg)
    {
        LOG(Error) << "Please override listenerCallback" << std::endl;
        //To ignore warning
        UNUSED(msg);

    }
    bool publishMsg(typename std_msgs::msg::Empty::SharedPtr msg)
    {
        UNUSED(msg);
        return true;
    }

};
}


