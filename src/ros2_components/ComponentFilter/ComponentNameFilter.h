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

#include "ComponentListFilter.h"
namespace ros2_components {


class ComponentNameFilter : public ComponentListFilter
{
public:
    ComponentNameFilter(std::string _Name);
    std::string getName() const;
    void setName(const std::string &value);

     virtual std::vector<ComponentInfo> filter(std::vector<ComponentInfo> infos);

private:
    std::string Name;
};
}

