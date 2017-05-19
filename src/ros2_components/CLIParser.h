#ifndef CLIPARSER_H
#define CLIPARSER_H

#include <string>
#include <vector>
#include <list>
#include <QString>
#include "CLIArgument.h"
namespace ros2_components
{
/**
 * @brief The CLIParser class is used to interpret passed commandline flags and parameters. The @ref CLIArgument are used to register arguments.
 * Additionally the parser implements a help argument (--help) that prints a custom help string (given in the constructor) and all registered arguments.
 */
class CLIParser
{
public:
    /**
     * @brief CLIParser
     * @param argv - String array with command line arguments
     * @param argc - Number of arguments
     * @param _helpString - Additional help string to be printed when --help was found
     */
    CLIParser(char* argv[], int argc, std::__cxx11::string _helpString);
    /**
     * @brief parse - Start parsing the arguments
     */
    void parse();
    /**
     * @brief registerArgument - Register a new Argument
     * @param arg
     */
    void registerArgument(CLIArgument::SharedPtr arg);
    /**
     * @brief printHelp - Print the help output
     * @param additionalInformation
     */
    void printHelp(std::__cxx11::string additionalInformation);
    /**
     * @brief getHelpFound
     * @return True if --help was found.
     */
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
