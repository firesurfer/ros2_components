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

#ifndef ENTITYWIDGETCASTEXCEPTION
#define ENTITYWIDGETCASTEXCEPTION

#include <exception>
#include <string>

class EntityWidgetCastException: public std::exception
{
public:
    explicit EntityWidgetCastException(const std::string& msg = "Could not cast entity widget to given type");

    virtual const char* what() const noexcept
    {
        return message.c_str();
    }
private:
    std::string message;
};

#endif //ENTITYWIDGETCASTEXCEPTION
