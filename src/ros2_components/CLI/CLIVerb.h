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

#ifndef CLIVERB_H
#define CLIVERB_H

#include <memory>
#include <string>
#include <map>
#include <vector>
#include "CLIArgument.h"
#include "CLIParameter.h"
#include <QString>
#include <list>
namespace  ros2_components {

/**
 * @brief The CLIVerb class represents a verb for the @ref CLIParser.
 * Example: <programname> build
 * Verbs can be nested
 */
class CLIVerb
{
public:
    typedef std::shared_ptr<CLIVerb> SharedPtr;

    CLIVerb();
    CLIVerb(std::string _verb, std::string _description);
    CLIVerb(std::string _verb, std::string _description, bool *_found);
    /**
     * @brief addVerb - add a sub verb to this verb
     * @param _child
     * @return
     */

    bool addVerb(CLIVerb& _child);
    /**
     * @brief addArgument - add an argument to this verb
     * @param _arg
     * @return
     */

    bool addArgument(CLIArgument& _arg);

    /**
     * @brief addParameter - add a parameter to this verb
     * @param _param
     * @return
     */

    bool addParameter(CLIParameter& _param);
    /**
     * @brief parse - Parse the corresponding arguments for this verb -> Pass subverbs and arguments to childen
     * @param arguments
     * @return
     */
    bool parse(std::vector<std::string>& str, bool *error);

    /**
     * @brief wasFound
     * @return true if the verb was found
     */
    bool wasFound();
    /**
     * @brief getName
     * @return The name of this verb
     */
    std::string getName() const;

    /**
     * @brief getDescription
     * @return The description string of this verb
     */
    std::string getDescription() const;
    /**
     * @brief getAllCliArguments
     * @return
     */
    std::vector<CLIArgument> getAllCliArguments() const;
    /**
     * @brief getChildVerbs
     * @return
     */
    std::map<std::string, CLIVerb> getChildVerbs() const;
    /**
     * @brief getAllCliParameter
     * @return
     */
    std::vector<CLIParameter> getAllCliParameter() const;


private:

    std::string name;
    std::string description;

    bool* found;

    bool isVerb(std::string arg);



    std::list<CLIArgument> cliArguments;
    std::list<CLIParameter> cliParameters;
    std::map<std::string, CLIVerb> childVerbs;

    std::vector<CLIArgument> allCliArguments;
    std::vector<CLIParameter> allCliParameters;

};
}

#endif // CLIVERB_H
