/*
 * Copyright 2018 <Lennart Nachtigall> <firesurfer127@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "CLIParser.h"

namespace ros2_components
{
CLIParser::CLIParser(char *argv[], int argc, std::string _programDescription):baseVerb{std::string(argv[0]), _programDescription}, helpArgument{"help", "Print this help", &helpFound}
{

    for(int i=0; i < argc;i++)
    {
        this->arguments.push_back(std::string(argv[i]));
    }
    //0 arg is the program name
    baseVerb.addArgument(helpArgument);
}

void CLIParser::parse()
{
    if(arguments.size() <= 0)
        return;
    std::vector<std::string> arg_copy = arguments;
    arg_copy.erase(arg_copy.begin());
    bool error = false;
    this->baseVerb.parse(arguments, &error);

    if(helpFound || error)
    {
        if(error)
        {
            std::cout << printInColor("Bad console argument!", ConsoleColor::FG_RED) << std::endl;
        }
        printHelp(this->helpString);
        exit(0);
    }
}

void CLIParser::addVerb(CLIVerb& verb)
{
    this->baseVerb.addVerb(verb);
}

void CLIParser::addArgument(CLIArgument &arg)
{
    this->baseVerb.addArgument(arg);
}

void CLIParser::printHelp(std::string additionalInformation)
{
    int depth = 0;
    std::cout <<  additionalInformation << std::endl;
    std::function<void(CLIVerb&)> func = [&](CLIVerb& verb)
    {
        //if(!verb)
            //return;
        for(int i = 0; i < depth; i++)
            std::cout << " ";

        std::cout << printInColor(verb.getName(),ConsoleColor::FG_BLUE) << std::endl;

        for(int i = 0; i < depth+4; i++)
            std::cout << " ";

        std::cout << verb.getDescription() << std::endl << std::endl;
        if(verb.getAllCliParameter().size() > 0)
        {
            for(int i = 0; i < depth+2; i++)
                std::cout << " ";
            std::cout << "Needed parameters: " << std::endl;
        }

        for(auto cliParam : verb.getAllCliParameter())
        {
            for(int i = 0; i < depth+4; i++)
                std::cout << " ";
            std::string output =  cliParam.getName() + "                     ";
            output.insert(20, cliParam.getDescription());
            std::cout << output << std::endl;
        }

        for(auto cliArg : verb.getAllCliArguments())
        {
            for(int i = 0; i < depth+4; i++)
                std::cout << " ";
            std::string output = " --" + cliArg.getName() + "                     ";
            output.insert(20, cliArg.getDescription());
            std::cout << output << std::endl;
        }

        depth = depth+2;
        for(auto subVerbEntry : verb.getChildVerbs())
        {
            CLIVerb& subVerb = subVerbEntry.second;

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

CLIVerb& CLIParser::getBaseVerb()
{
    return baseVerb;
}
}
