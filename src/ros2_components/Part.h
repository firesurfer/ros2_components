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
namespace ros2_components
{
template <typename MessageType>
class Part : public Entity<MessageType>
{

public:
    Part( int64_t _id, bool _subscribe ,std::shared_ptr<rclcpp::Node> parentNode,std::string name) : Entity<MessageType>(_id,_subscribe,parentNode, name)
    {

    }

    template<typename T>
    std::shared_ptr<T> getConcreteChildById(int id)
    {
        std::shared_ptr<T> con_ch;
        con_ch = dynamic_pointer_cast<T>(this->getChildById(id));
        if(con_ch == NULL)
            throw EntityCastException("Wrong type - can't cast");
        return con_ch;
    }

};

}


