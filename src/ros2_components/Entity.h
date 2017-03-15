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
#include "EntityBase.h"

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
    Entity(int64_t _id, bool _subscribe, std::shared_ptr<rclcpp::node::Node> parentNode, std::string className) : EntityBase(_id, _subscribe, parentNode, className)
    {
        //Some ROS2 QOS Configuration -> Taken from an example
        custom_qos_profile = rmw_qos_profile_sensor_data;
        //custom_qos_profile.depth = 2;
        //custom_qos_profile.history = hist_pol;
        if(!isSubscriber())
        {
            entityPublisher = parentNode->create_publisher<MessageType>(getName(), custom_qos_profile);

            //rmw_qos_profile_services_default
            rmw_qos_profile_t component_manager_profile = rmw_qos_profile_parameters;
            component_manager_profile.depth = 1000;
            //component_manager_profile.history = RMW_QOS_POLICY_KEEP_ALL_HISTORY;
            pubBase = entityPublisher;

        }
        else
        {
            using namespace std::placeholders;
            entitySubscription = parentNode->create_subscription<MessageType>(getName(), std::bind(&Entity::internalListenerCallback, this,_1), custom_qos_profile);
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
        LOG(Error) << "Entity:Please override publish function" << std::endl;
        return true;
    }


    /**
     * @brief add a new listener to be called when new data arrives
     */
    void addListener(std::function<void(typename MessageType::SharedPtr)> listener)
    {
        listeners.push_back(listener);
    }



protected:
    /**
     * @brief This is the method to handle new data inside your entity
     */
    virtual void listenerCallback(const typename MessageType::SharedPtr  msg)
    {
        //To ignore warning
        UNUSED(msg);

    }
    bool publishMsg(typename MessageType::SharedPtr msg)
    {
        if(!this->isSubscriber())
            if(this->entityPublisher)
                entityPublisher->publish(msg);
            else
                return false;
        else return false;
        return true;
    }

    //ROS 2 Stuff
    rmw_qos_profile_t custom_qos_profile;

    std::shared_ptr<rclcpp::publisher::Publisher<MessageType>> entityPublisher;
    std::shared_ptr<rclcpp::subscription::Subscription<MessageType>> entitySubscription;
    rclcpp::subscription::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameterEventSubscription;




private:
    std::vector<std::function<void(typename MessageType::SharedPtr)>> listeners;
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
        }
    }

};

}


