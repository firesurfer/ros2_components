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

#ifndef CLIPARAMETER_H
#define CLIPARAMETER_H

#include <string>
#include <memory>
#include <iostream>
namespace ros2_components
{
/**
 * @brief The CLIParameter class represents a parameter of a verb. Forexample you provide a verb named build for you program you might want to add a parameter for a package or source file name.
 * This would lead to the following syntax:
 * <programname> build <filename>
 * The parameters for a verb are order sensitive. That means if you add multiple parameters they have to be supplied in the order you have added them.
 */
class CLIParameter
{
public:
    typedef std::shared_ptr<CLIParameter> SharedPtr;
    /**
     * @brief CLIParameter
     * @param _name
     * @param _description
     * @param _param
     * @param optional - Optional is not implemented yet!
     */
    CLIParameter(std::string _name, std::string _description, std::string* _param, bool _optional = false);
    /**
     * @brief parse
     * @param parameter
     * @return true if successfully parsed. The parameter implemented currently always returns true
     * @exception May throw a std::invalid_argument exception if std::string* param was NULL.
     */
    bool parse(std::string parameter);
    /**
     * @brief getName
     * @return The name of the parameter.
     */
    std::string getName() const;

    /**
     * @brief getDescription
     * @return The description of the parameter
     */
    std::string getDescription() const;
    /**
     * @brief getOptional
     * @return True in case this parameter is optional
     */
    bool getOptional() const;

private:
    std::string name;
    std::string description;
    std::string* param;
    bool optional;

};
}

#endif // CLIPARAMETER_H
