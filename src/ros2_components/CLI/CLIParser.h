#ifndef CLIPARSER_H
#define CLIPARSER_H

#include <string>
#include <vector>
#include <list>
#include <QString>
#include "CLIArgument.h"
#include "CLIVerb.h"
#include <functional>
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
    CLIParser(char* argv[], int argc, std::__cxx11::string _programDescription);
    /**
     * @brief parse - Start parsing the arguments
     */
    void parse();
    /**
     * @brief addArgument - Add argument to the baseVerb
     * @param arg
     */
    void addArgument(CLIArgument::SharedPtr arg);
    /**
     * @brief addVerb - Add verb to the baseVerb. In case you want to nest verbs add the nested verbs to the subverb you pass to this function.
     * @param verb
     */
    void addVerb(CLIVerb::SharedPtr verb);
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

    /**
     * @brief getBaseVerb
     * @return
     */
    CLIVerb::SharedPtr getBaseVerb() const;

private:
    /**
     * @brief baseVerb - Base Verb with the same name as the application
     */
    CLIVerb::SharedPtr baseVerb;
    std::vector<std::string> arguments;


    bool helpFound = false;
    std::string helpString;

};
}

#endif // CLIPARSER_H
