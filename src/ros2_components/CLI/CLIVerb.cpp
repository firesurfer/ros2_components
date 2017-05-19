#include "CLIVerb.h"

namespace  ros2_components {

CLIVerb::CLIVerb(std::__cxx11::string _verb, std::__cxx11::string _description, std::shared_ptr<CLIVerb> _parent)
{
    this->name = _verb;
    this->description = _description;
    this->parent = _parent;

}

CLIVerb::CLIVerb(std::__cxx11::string _verb, std::__cxx11::string _description, std::shared_ptr<CLIVerb> _parent, bool *_found):CLIVerb(_verb,_description,_parent)
{
    this->found = _found;
    if(found != nullptr)
    {
        *(this->found) = false;
    }
}

bool CLIVerb::addVerb(CLIVerb::SharedPtr _child)
{
    this->childVerbs[_child->getName()] = _child;
}

bool CLIVerb::addArgument(CLIArgument::SharedPtr _arg)
{
    this->cliArguments.push_back(_arg);
    this->allCliArguments.push_back(_arg);
}

bool CLIVerb::parse(std::vector<std::__cxx11::string> &str)
{
    std::vector<std::string> arguments;

    for(auto it = str.begin(); it != str.end();)
    {
        std::string arg = *it;

        if(isVerb(arg))
        {
            if(arg == this->name)
                if(found != nullptr)
                    *this->found = true;
            if(childVerbs[arg] != nullptr)
                childVerbs[arg]->parse(str);
            it++;
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

bool CLIVerb::isVerb(std::__cxx11::string arg)
{
    QString qarg = QString::fromStdString(arg);

    return !(qarg.startsWith("-") || qarg.startsWith("--"));


}

}
