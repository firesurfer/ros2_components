#include "CLIParser.h"

namespace ros2_components
{
CLIParser::CLIParser(char *argv[], int argc, std::string _programDescription)
{

    for(int i=0; i < argc;i++)
    {
        this->arguments.push_back(std::string(argv[i]));

    }
    //0 arg is the program name
    this->baseVerb = std::make_shared<CLIVerb>(arguments[0], _programDescription,nullptr);
    this->baseVerb->addArgument(std::make_shared<CLIArgument>("help", "Print this help", &helpFound ));
}

void CLIParser::parse()
{
    arguments.erase(arguments.begin());
    this->baseVerb->parse(arguments);


    if(helpFound)
    {
        printHelp(this->helpString);
    }


}

void CLIParser::addVerb(CLIVerb::SharedPtr verb)
{
    this->baseVerb->addVerb(verb);
}

void CLIParser::addArgument(CLIArgument::SharedPtr arg)
{
    this->baseVerb->addArgument(arg);
}

void CLIParser::printHelp(std::string additionalInformation)
{

    int depth = 0;
    std::cout <<  additionalInformation << std::endl;
    std::function<void(CLIVerb::SharedPtr)> func = [&](CLIVerb::SharedPtr verb){


        for(int i = 0; i < depth; i++)
            std::cout << " ";
        std::cout << printInColor(verb->getName(),ConsoleColor::FG_BLUE) << std::endl;
        for(int i = 0; i < depth+4; i++)
            std::cout << " ";
        std::cout << verb->getDescription() << std::endl << std::endl;
        if(verb->getAllCliParameter().size() > 0)
        {
            for(int i = 0; i < depth+2; i++)
                std::cout << " ";
            std::cout << "Needed parameters: " << std::endl;
        }
        for(auto cliParam : verb->getAllCliParameter())
        {
            for(int i = 0; i < depth+4; i++)
                std::cout << " ";
            std::string output =  cliParam->getName() + "                     ";
            output.insert(20, cliParam->getDescription());
            std::cout << output << std::endl;
        }
        for(auto cliArg : verb->getAllCliArguments())
        {
            for(int i = 0; i < depth+4; i++)
                std::cout << " ";
            std::string output = " --" + cliArg->getName() + "                     ";
            output.insert(20, cliArg->getDescription());
            std::cout << output << std::endl;
        }



         depth = depth+2;
        for(auto subVerbEntry : verb->getChildVerbs())
        {
            CLIVerb::SharedPtr subVerb = subVerbEntry.second;

            func(subVerb);
        }
        depth = depth-2;


    };
    func(this->baseVerb);


}

bool CLIParser::getHelpFound() const
{
    return helpFound;
}

CLIVerb::SharedPtr CLIParser::getBaseVerb() const
{
    return baseVerb;
}
}
