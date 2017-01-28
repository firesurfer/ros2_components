/*
 * Copyright 2016 <Lennart Nachtigall> <firesurfer65@yahoo.de>
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

#ifndef COMPONENTINFO_H
#define COMPONENTINFO_H

#include <QObject>
/**
 * Represents information about a certain component (e.g a motor)
 * These information allow to  definitely identify any component/entity in the system
 *
 * TODO Move the name from ComponentInfo to EntityInfo
 */
namespace ros2_components
{
struct ComponentInfo
{

public:
    ComponentInfo();
    //Copy constructor
    ComponentInfo(const ComponentInfo &info);
    uint64_t id;
    std::string type;
    std::string name;
    int64_t parentId;
    std::string parentType;
    std::vector<int64_t> childIds;
    std::vector<std::string> childTypes;
    int64_t machineip;
    std::string nodename;

};

}

#endif // COMPONENTINFO_H
