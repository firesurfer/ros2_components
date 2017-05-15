#include "CLIParser.h"

namespace ros2_components
{
CLIParser::CLIParser(char *argv[], int argc, std::string _helpString)
{
    helpString = _helpString;
    for(int i=0; i < argc;i++)
    {
        this->arguments.push_back(std::string(argv[i]));

    }
    registerArgument(std::make_shared<CLIArgument>("help", "Print this help", &helpFound ));
}

void CLIParser::parse()
{
    for(std::string & arg: arguments)
    {
        for(CLIArgument::SharedPtr & cliArg: cliArguments)
        {
            if(cliArg->check(arg))
            {
                if(cliArg->getName() != "help")
                {
                    this->cliArguments.remove(cliArg);
                    break;
                }
            }
        }
        if(helpFound)
            printHelp(this->helpString);

    }
}

void CLIParser::registerArgument(CLIArgument::SharedPtr arg)
{
    this->cliArguments.push_back(arg);
    this->allCliArguments.push_back(arg);
}

void CLIParser::printHelp(std::string additionalInformation)
{
    std::cout <<  additionalInformation << std::endl;
    for(CLIArgument::SharedPtr & cliArg: cliArguments)
    {
        std::string output = " --" + cliArg->getName() + "                     ";
        output.insert(20, cliArg->getDescription());

        std::cout << output << std::endl;
    }
}

bool CLIParser::getHelpFound() const
{
    return helpFound;
}
}
