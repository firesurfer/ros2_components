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

#ifndef UNIT_H
#define UNIT_H

#include "ros2_components/Entity.h"
/**
 * This class represents a unit of an actor and a sensor. Usefull e.g. if you have a part of your robot e.g. a joint
 * that can be used for setting commands to the hardware and reading parameter from the hardware
 * The unit itself can also publish a message and can contain meta information
 */
namespace KamaroModule
{
    template <typename MessageType, typename ActorType, typename SensorType>
    class Unit :  public Entity<MessageType>
    {
    public:
	Unit( int32_t _id, bool _subscribe ,std::shared_ptr<rclcpp::node::Node> parentNode,std::string name,std::shared_ptr<ActorType> actor, std::shared_ptr<SensorType> sensor) : Entity<MessageType>(_id,_subscribe,parentNode, name)
	{
	    
	    this->addChild(actor);
	    this->addChild(sensor);
	}
	std::shared_ptr<ActorType> getActor()
	{
	    std::shared_ptr<ActorType> actor = dynamic_pointer_cast<ActorType>(this->getChild(0));
	    if(actor == NULL)
		throw std::runtime_error("Could not cast actor");
	    return  actor;
	}
	std::shared_ptr<SensorType> getSensor()
	{
	     std::shared_ptr<SensorType> sensor = dynamic_pointer_cast<SensorType>(this->getChild(1));
	    if(sensor == NULL)
		throw std::runtime_error("Could not cast sensor");
	    return  sensor;
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
#endif // UNIT_H
