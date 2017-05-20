#ifndef CLIPARAMETER_H
#define CLIPARAMETER_H

#include <string>
#include <memory>
namespace ros2_components
{

class CLIParameter
{
public:
    typedef std::shared_ptr<CLIParameter> SharedPtr;
    CLIParameter(std::string _name, std::string _description, std::string* _param);
    bool parse(std::string parameter);
    std::string getName() const;

    std::string getDescription() const;

private:
    std::string name;
    std::string description;
    std::string* param;

};
}

#endif // CLIPARAMETER_H
