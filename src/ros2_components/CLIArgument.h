#ifndef CLIARGUMENT_H
#define CLIARGUMENT_H

#include <memory>
#include <string>
#include <iostream>
#include <QString>
namespace ros2_components
{
class CLIArgument
{
public:
    typedef std::shared_ptr<CLIArgument> SharedPtr;
    CLIArgument(std::string _name, std::string _description, bool * _found);
    CLIArgument(std::string _name, std::string _description, std::string *_argument);
    bool check(std::string element);
    std::string getDescription() const;

    std::string getName() const;

private:
    bool * found;
    std::string * argument;
    bool isFlag;
    std::string name;
    std::string description;
};

}
#endif // CLIARGUMENT_H
