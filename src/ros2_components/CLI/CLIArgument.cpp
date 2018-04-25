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

#include "CLIArgument.h"

namespace ros2_components
{

CLIArgument::CLIArgument(std::__cxx11::string _name, std::string _description, bool *_found)
{
    this->name = _name;
    this->description = _description;
    this->isFlag = true;
    this->found = _found;
    if(found != nullptr)
        *this->found = false;
    this->argument = nullptr;
}

CLIArgument::CLIArgument(std::__cxx11::string _name, std::string _description, std::__cxx11::string *_argument)
{
    this->name = _name;
    this->description = _description;
    this->isFlag = false;
    this->argument = _argument;
    if(argument != nullptr)
        *this->argument = "";
    this->found = nullptr;
}

bool CLIArgument::parse(std::__cxx11::string element)
{
    if(isFlag)
    {
        QString qstr = QString::fromStdString(element);
        if(qstr.startsWith("--"))
            qstr = qstr.remove(0,2);
        if(qstr.trimmed().toStdString() == name)
        {
            if(found != nullptr)
                *found = true;
            else
                throw std::invalid_argument("found ptr was NULL in: " + name);
            return true;
        }
    }
    else
    {
        QString qstr = QString::fromStdString(element);
        if(qstr.startsWith("--"))
            qstr = qstr.remove(0,2);
        int pos = qstr.indexOf("=");
        QString namestr = qstr.mid(0, pos);
        if(namestr.trimmed().toStdString() == name)
        {
            qstr = qstr.mid(pos+1, qstr.length()- pos);
            if(argument != nullptr)
                *this->argument = qstr.toStdString();
            else
                throw std::invalid_argument("argument ptr was NULL in: " + name);
            return true;
        }
    }
    return false;
}

std::string CLIArgument::getDescription() const
{
    return description;
}

std::string CLIArgument::getName() const
{
    return name;
}
}
