#include "CLIParameter.h"


namespace  ros2_components {


CLIParameter::CLIParameter(std::string _name, std::string _description, std::string* _param)
{
    this->name = _name;
    this->description = _description;
    this->param = _param;
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

}
