#ifndef CLIPARSER_H
#define CLIPARSER_H

#include <string>
#include <vector>
#include <list>
#include <QString>
#include "CLIArgument.h"
namespace ros2_components
{
class CLIParser
{
public:
    CLIParser(char* argv[], int argc, std::__cxx11::string _helpString);
    void parse();
    void registerArgument(CLIArgument::SharedPtr arg);
    void printHelp(std::__cxx11::string additionalInformation);
    bool getHelpFound() const;

private:
    std::vector<std::string> arguments;
    std::list<CLIArgument::SharedPtr> cliArguments;
    std::vector<CLIArgument::SharedPtr> allCliArguments;
    bool helpFound = false;
    std::string helpString;

};
}

#endif // CLIPARSER_H
