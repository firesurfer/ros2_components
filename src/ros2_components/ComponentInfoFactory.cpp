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

#include "ComponentInfoFactory.h"
namespace ros2_components
{
ComponentInfoFactory::ComponentInfoFactory()
{

}

ComponentInfo ComponentInfoFactory::fromEntity(EntityBase::SharedPtr ent)
{
    ComponentInfo info;
    info.name = ent->getName();
    info.id = ent->getId();

    //TODO move this into a more general place

    info.machineip = Networking::getLocalIpV4();
    if(ent->getParent() != nullptr)
    {
        info.parentId = ent->getParent()->getId();
        info.parentType = ent->getParent()->getClassName();
    }
    else
    {
        info.parentId = -1;
        info.parentType = "";
    }
    info.type = ent->getClassName();

    for(EntityBase::SharedPtr & child : ent->getAllChilds())
    {
        info.childIds.push_back(child->getId());
        info.childTypes.push_back(child->getClassName());
    }
    info.name = ent->getName();
    info.subscriber = ent->isSubscriber();

    return info;

}

ComponentInfo ComponentInfoFactory::fromListComponentsResponseMessage(ros2_components_msg::msg::ListComponentsResponse::SharedPtr msg)
{
    ComponentInfo info;
    info.id = msg->id;
    info.name = msg->componentname;
    info.parentId = msg->parent;
    info.parentType = msg->parenttype;
    info.childIds = msg->childids;
    info.childTypes = msg->childtypes;
    info.nodename = msg->nodename;
    info.type = msg->type;
    info.machineip = msg->machineip;
    info.subscriber = msg->subscriber;
    //TODO check if all fields are used
    return info;
}

ComponentInfo ComponentInfoFactory::fromComponentChangedMessage(ros2_components_msg::msg::ComponentChanged::SharedPtr msg)
{
    ComponentInfo info;
    info.id = msg->id;
    info.name = msg->componentname;
    info.parentId = msg->parent;
    info.parentType = msg->parenttype;
    info.childIds = msg->childids;
    info.childTypes = msg->childtypes;
    info.nodename = msg->nodename;
    info.type = msg->type;
    info.machineip = msg->machineip;
    info.subscriber = msg->subscriber;
    //TODO check if all fields are used
    return info;
}


}
