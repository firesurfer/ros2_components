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

#include "Entity.h"
#include "std_msgs/msg/empty.hpp"

namespace ros2_components {

/**
 * @brief The NodeEntity class represents a node in the component system.
 */

class NodeEntity: public Entity<std_msgs::msg::Empty>
{

public:
    typedef std::shared_ptr<NodeEntity> SharedPtr;
    NodeEntity(uint64_t _id, std::string _name, NodeContainer::SharedPtr _nodeContainer);

    virtual ~NodeEntity();

    std::string getNodeName() const;

private:
    std::string node_name;
};
}

