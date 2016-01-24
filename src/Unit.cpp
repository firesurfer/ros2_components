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

#include "Unit.h"
namespace KamaroModule {
    
    Unit::Unit( uint16_t _id, bool _subscribe ,std::shared_ptr<rclcpp::node::Node> parentNode,std::shared_ptr<ActorType> actor, std::shared_ptr<SensorType> sensor) : Entity<MessageType>(_id,_subscribe,parentNode, "SensorLidar")
    {
	this->addChild(actor);
	this->addChild(sensor);
    }
    
}