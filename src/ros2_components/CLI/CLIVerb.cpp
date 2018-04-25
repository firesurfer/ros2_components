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

#include "CLIVerb.h"

namespace  ros2_components {






CLIVerb::CLIVerb()
{

}

CLIVerb::CLIVerb(std::__cxx11::string _verb, std::__cxx11::string _description)
{
    this->name = _verb;
    this->description = _description;
    this->found = nullptr;

}

CLIVerb::CLIVerb(std::__cxx11::string _verb, std::__cxx11::string _description, bool *_found):CLIVerb(_verb,_description)
{
    this->found = _found;
    if(found != nullptr)
        *found = false;
}

bool CLIVerb::addVerb(CLIVerb &_child)
{
    if(this->childVerbs.find(_child.getName()) != this->childVerbs.end())
        return false;
    this->childVerbs[_child.getName()] = _child;
    return true;
}



bool CLIVerb::addArgument(CLIArgument &_arg)
{
    cliArguments.push_back(_arg);
    allCliArguments.push_back(_arg);
    return true;
}



bool CLIVerb::addParameter(CLIParameter &_param)
{
    this->cliParameters.push_back(_param);
    allCliParameters.push_back(_param);
    return true;
}

bool CLIVerb::parse(std::vector<std::__cxx11::string> &str, bool * error)
{
    if(str.size() <= 0)
        return false;
    std::vector<std::string> arguments;
    std::vector<std::string> parameters;
    unsigned int count = 0;
    for(auto it = str.begin(); it != str.end();)
    {

        std::string arg = *it;

        if(arg == "")
        {
            str.erase(it);
            continue;
        }
        //First check if this arg was found
        if(arg == this->name)
        {
            if(found != nullptr)
                *this->found = true;
            str.erase(it);
            continue;
        }
        //Second check for parameters
        if(count < this->cliParameters.size())
        {
            parameters.push_back(arg);
            str.erase(it);
            count++;
            continue;
        }


        if(isVerb(arg))
        {
            if(arg != "")
            {
                if(childVerbs.find(arg) != childVerbs.end())
                    childVerbs[arg].parse(str,error);
                else
                {
                    str.erase(it);
                    *error = true;
                }
            }
        }
        else
        {
            QString qarg = QString::fromStdString(arg);
            arguments.push_back(arg);
            str.erase(it);

        }
    }
    for(std::string & argument: arguments)
    {
        for(auto it=cliArguments.begin();it !=cliArguments.end();it++)
        {
            //std::cout << argument << std::endl;
            if(it->parse(argument))
            {
                if(it->getName() != "help")
                {
                    this->cliArguments.erase(it);
                    break;
                }
            }
        }

    }
    int i = 0;
    if(cliParameters.size() != parameters.size())
    {
        throw std::runtime_error("Wrong amount of parameters. Expecting: "+ std::to_string(cliParameters.size()));
    }

    for(CLIParameter& param: this->cliParameters)
    {
        param.parse(parameters[i]);
        i++;
    }
    return true;
}

bool CLIVerb::wasFound()
{
    return found;
}

std::string CLIVerb::getName() const
{
    return name;
}


std::string CLIVerb::getDescription() const
{
    return description;
}

std::vector<CLIArgument> CLIVerb::getAllCliArguments() const
{
    return allCliArguments;
}

std::map<std::string, CLIVerb> CLIVerb::getChildVerbs() const
{
    return childVerbs;
}

std::vector<CLIParameter> CLIVerb::getAllCliParameter() const
{
    return allCliParameters;
}

bool CLIVerb::isVerb(std::__cxx11::string arg)
{
    QString qarg = QString::fromStdString(arg);
    return !(qarg.startsWith("-") || qarg.startsWith("--"));
}


}
