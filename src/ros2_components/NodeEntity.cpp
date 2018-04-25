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

#include "NodeEntity.h"


namespace  ros2_components {


NodeEntity::NodeEntity(uint64_t _id, std::string _name,  NodeContainer::SharedPtr _nodeContainer):Entity<std_msgs::msg::Empty>(_id,true, _nodeContainer, "NodeEntity", _name)
{
    this->node_name = _nodeContainer->getRosNode()->get_name();
    REFLECT(node_name);
}

NodeEntity::~NodeEntity()
{

}

std::string NodeEntity::getNodeName() const
{
    return node_name;
}

}
