#include "CLIVerb.h"

namespace  ros2_components {

CLIVerb::CLIVerb(std::__cxx11::string _verb, std::__cxx11::string _description, std::shared_ptr<CLIVerb> _parent)
{
    this->name = _verb;
    this->description = _description;
    this->parent = _parent;
    this->found = nullptr;
}

CLIVerb::CLIVerb(std::__cxx11::string _verb, std::__cxx11::string _description, std::shared_ptr<CLIVerb> _parent, bool *_found):CLIVerb(_verb,_description,_parent)
{
    this->found = _found;
    if(found != nullptr)
        *(this->found) = false;
}

bool CLIVerb::addVerb(CLIVerb::SharedPtr _child)
{
    if(this->childVerbs.find(_child->getName()) != this->childVerbs.end())
        return false;
    this->childVerbs[_child->getName()] = _child;
    return true;
}

bool CLIVerb::addArgument(CLIArgument::SharedPtr _arg)
{
    //TODO return false if argument already in list
    this->cliArguments.push_back(_arg);
    this->allCliArguments.push_back(_arg);
    return true;
}

bool CLIVerb::addParameter(CLIParameter::SharedPtr _param)
{
    //TODO return fals if parameter is already in list
    this->cliParameters.push_back(_param);
    this->allCliParameter.push_back(_param);
}

bool CLIVerb::parse(std::vector<std::__cxx11::string> &str, bool * error)
{
    if(str.size() <= 0)
        return false;
    std::vector<std::string> arguments;
    std::vector<std::string> parameters;
    int count = 0;
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
        if(count < this->allCliParameter.size())
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
                if(childVerbs[arg] != nullptr)
                    childVerbs[arg]->parse(str,error);
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
        for(CLIArgument::SharedPtr & cliArg: cliArguments)
        {
            if(cliArg->parse(argument))
            {
                if(cliArg->getName() != "help")
                {
                    this->cliArguments.remove(cliArg);
                    break;
                }
            }
        }
    }
    int i = 0;
    if(allCliParameter.size() != parameters.size())
        throw std::runtime_error("Wrong amount of parameters. Expecting: "+ std::to_string(allCliParameter.size()));

    for(CLIParameter::SharedPtr param: this->allCliParameter)
    {
        param->parse(parameters[i]);
        i++;
    }
}

bool CLIVerb::wasFound()
{
    return found;
}

std::string CLIVerb::getName() const
{
    return name;
}

std::shared_ptr<CLIVerb> CLIVerb::getParent() const
{
    return parent;
}

std::string CLIVerb::getDescription() const
{
    return description;
}

std::vector<CLIArgument::SharedPtr> CLIVerb::getAllCliArguments() const
{
    return allCliArguments;
}

std::map<std::string, CLIVerb::SharedPtr> CLIVerb::getChildVerbs() const
{
    return childVerbs;
}

std::vector<CLIParameter::SharedPtr> CLIVerb::getAllCliParameter() const
{
    return allCliParameter;
}

bool CLIVerb::isVerb(std::__cxx11::string arg)
{
    QString qarg = QString::fromStdString(arg);
    return !(qarg.startsWith("-") || qarg.startsWith("--"));
}


}
