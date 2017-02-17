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

#include "ComponentNodeNameFilter.h"
namespace ros2_components
{
ComponentNodeNameFilter::ComponentNodeNameFilter(std::__cxx11::string _NodeName)
{
    this->NodeName = _NodeName;
}

std::string ComponentNodeNameFilter::getNodeName() const
{
    return NodeName;
}

void ComponentNodeNameFilter::setNodeName(const std::string &value)
{
    NodeName = value;
}

std::vector<ComponentInfo> ComponentNodeNameFilter::Filter(std::vector<ComponentInfo> infos)
{
    std::vector<ComponentInfo> components;
    for(ComponentInfo & info: infos)
    {
        if(info.nodename == this->NodeName)
            components.push_back(info);
    }
    return components;
}

}
