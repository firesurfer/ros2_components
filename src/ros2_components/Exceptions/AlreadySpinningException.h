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

#ifndef ALREADYSPINNINGEXCEPTION_H
#define ALREADYSPINNINGEXCEPTION_H

#include <exception>

class AlreadySpinningException: public std::exception
{
public:
    AlreadySpinningException();

    virtual const char* what() const noexcept
    {
       return "Node is already spinning";
    }
};

#endif // ALREADYSPINNINGEXCEPTION_H
