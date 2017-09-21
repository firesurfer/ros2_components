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

#include "ComponentNameFilter.h"
namespace ros2_components
{
ComponentNameFilter::ComponentNameFilter(std::__cxx11::string _Name)
{
    Name = _Name;
}

std::string ComponentNameFilter::getName() const
{
    return Name;
}

void ComponentNameFilter::setName(const std::string &value)
{
    Name = value;
}

std::vector<ComponentInfo> ComponentNameFilter::filter(std::vector<ComponentInfo> infos)
{
    std::vector<ComponentInfo> components;
    for(ComponentInfo & info: infos)
    {
        if(info.name == this->Name)
            components.push_back(info);
    }
    return components;
}

}
