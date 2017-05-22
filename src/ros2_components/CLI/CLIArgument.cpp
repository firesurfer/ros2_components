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
