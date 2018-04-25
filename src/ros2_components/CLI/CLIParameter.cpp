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

#include "CLIParameter.h"
#include <cassert>

namespace  ros2_components {


CLIParameter::CLIParameter(std::string _name, std::string _description, std::string* _param, bool _optional)
{
    this->name = _name;
    this->description = _description;
    this->param = _param;
    this->optional = _optional;
    assert(optional != true);
}

bool CLIParameter::parse(std::__cxx11::string parameter)
{
    if(param != nullptr)
        *param = parameter;
    else
        throw std::invalid_argument("param was NULL");
    return true;
}

std::string CLIParameter::getName() const
{
    return name;
}

std::string CLIParameter::getDescription() const
{
    return description;
}

bool CLIParameter::getOptional() const
{
    return optional;
}

}
