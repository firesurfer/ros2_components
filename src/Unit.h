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
    template <typename MessageType, ActorType, SensorType>
    class Unit :  public Entity<MessageType>
    {
    public:
	Unit( uint16_t _id, bool _subscribe ,std::shared_ptr<rclcpp::node::Node> parentNode, std::shared_ptr<ActorType> actor, std::shared_ptr<SensorType> sensor);
	std::shared_ptr<ActorType> getActor()
	{
	    return childs[0];
	}
	std::shared_ptr<SensorType> getSensor()
	{
	    return childs[1];
	}
    
	
	
    };
    
}
#endif // UNIT_H
