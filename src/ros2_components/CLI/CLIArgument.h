#ifndef CLIARGUMENT_H
#define CLIARGUMENT_H

#include <memory>
#include <string>
#include <iostream>
#include <QString>
namespace ros2_components
{
/**
 * @brief The CLIArgument class represents a single argument for the command line.
 * It distinguishes between flags and parameters. For detecting a passed flag use the constructor with "bool *_found".
 * For detecting an parameter use the second constructor with "std::string * _argument".
 * You need to register the argument at a CLIParser instance.
 */
class CLIArgument
{
public:
    typedef std::shared_ptr<CLIArgument> SharedPtr;
    CLIArgument(std::string _name, std::string _description, bool * _found);
    CLIArgument(std::string _name, std::string _description, std::string *_argument);
    bool parse(std::string element);
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
