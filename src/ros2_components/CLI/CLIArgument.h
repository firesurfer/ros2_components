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
 * You need to register the argument at a verb.
 * Examples for a valid flag:
 * <programname> -v
 *
 * Examples for a valid parameter
 * <programname> --name=hallo
 */
class CLIArgument
{
public:
    typedef std::shared_ptr<CLIArgument> SharedPtr;
    /**
     * @brief CLIArgument - Create a flag.
     * @param _name
     * @param _description
     * @param _found will be set to true in case the flag was found
     */
    CLIArgument(std::string _name, std::string _description, bool * _found);
    /**
     * @brief CLIArgument - Create a parameter (e.g. --name=<value>)
     * @param _name
     * @param _description
     * @param _argument will be set to parsed argument
     */
    CLIArgument(std::string _name, std::string _description, std::string *_argument);
    /**
     * @brief parse - parse the given element for this parameter
     * @param element
     * @return True if found.
     */
    bool parse(std::string element);
    /**
     * @brief getDescription
     * @return The argument description
     */
    std::string getDescription() const;

    /**
     * @brief getName
     * @return The name
     */
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
