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

#include "Entity.h"

#include "ros2_components_exceptions.h"
/**
 * This class represents a unit of an actor and a sensor. Usefull e.g. if you have a part of your robot e.g. a joint
 * that can be used for setting commands to the hardware and reading parameter from the hardware
 * The unit itself can also publish a message and can contain meta information
 */
namespace ros2_components
{
template <typename MessageType, typename ActorType, typename SensorType>
class Unit :  public Entity<MessageType>
{

public:
    Unit( int32_t _id, bool _subscribe ,std::shared_ptr<rclcpp::Node> parentNode,std::string name) : Entity<MessageType>(_id,_subscribe,parentNode, name)
    {

    }
    void addActor(std::shared_ptr<ActorType> actor)
    {
        this->addChild(actor);
    }
    void addSensor(std::shared_ptr<SensorType> sensor)
    {
        this->addChild(sensor);
    }
    std::shared_ptr<ActorType> getFirstActor()
    {
        for(auto & child: this->getAllChilds())
        {
            std::shared_ptr<ActorType> actor = dynamic_pointer_cast<ActorType>(child);
            if(actor != NULL)
                return  actor;
        }

        throw MissingChildException("Could not find any actor");

    }
    std::shared_ptr<SensorType> getFirstSensor()
    {
        for(auto & child: this->getAllChilds())
        {
            std::shared_ptr<SensorType> sensor = dynamic_pointer_cast<SensorType>(child);
            if(sensor != NULL)
                return  sensor;
        }
        throw MissingChildException("Could not find any sensor");

    }
    virtual bool publish()
    {
        for(auto & child : this->childs)
        {
            if(!child->isSubscriber())
                child->publish();
        }
        return true;
    }


};

}

