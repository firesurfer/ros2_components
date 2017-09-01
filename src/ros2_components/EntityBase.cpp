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

#include "EntityBase.h"
namespace ros2_components {

EntityBase::EntityBase(int64_t _id, bool _subscribe, std::shared_ptr<rclcpp::node::Node> _parentNode, string _className)
{
    this->id = _id;
    this->subscriber = _subscribe;
    this->parentNode = _parentNode;
    this->className = _className;
    this->active = true;
    this->name = getClassName()  + std::to_string(id);

    //TODO rework meta mechanism

    REFLECT(virtualEntity);
    REFLECT(className);
    REFLECT(active);
    REFLECT(description)
    qRegisterMetaType<int64_t>("int64_t");
    qRegisterMetaType<std::string>("std::string");


}

EntityBase::EntityBase(int64_t _id, bool _subscribe, std::shared_ptr<rclcpp::node::Node> _parentNode, string _className, string _componentName):EntityBase(_id,_subscribe, _parentNode,_className)
{
    this->name = _componentName + std::to_string(_id);
}

EntityBase::~EntityBase()
{
    std::cout << "Destroying: " << getName() << std::endl;
   /*ComponentInfo info;
    info.id = getId();
    info.parentId = parent->getId();
    info.name = getName();
    info.nodename = parentNode->get_name();
    info.parentType = parent->getClassName();
    info.type = getClassName();*/

    //emit entityDeleted(info);
}

int64_t EntityBase::getId()
{
    return id;
}

string EntityBase::getName()
{
    return name;
}


string EntityBase::getClassName()
{
    /* const QMetaObject* metaObject = this->metaObject();

    std::string localClassName = metaObject->className();
    localClassName.erase(0, localClassName.find_last_of(":")+1);*/
    return className;
}

bool EntityBase::isVirtual()
{
    return virtualEntity;
}

bool EntityBase::isSubscriber()
{
    return subscriber;
}

rclcpp::node::Node::SharedPtr EntityBase::getParentNode()
{
    return parentNode;
}

void EntityBase::addChild(std::shared_ptr<EntityBase> child, bool remote)
{
    LOG(LogLevel::Debug) << "addChild called with: " << child->getName() << " from: " << getName()<< std::endl;
    childs.push_back(child);
    child->setParent(shared_from_this());
    emit childAdded(child,child->getParent(),remote);
}

void EntityBase::removeChild(std::shared_ptr<EntityBase> child, bool remote)
{
    auto iteratorPos = std::find(childs.begin(), childs.end(), child) ;
    if(iteratorPos != childs.end())
    {
        childs.erase(iteratorPos);
    }
    else
        throw std::runtime_error("Can't remove given child - child not found!");
    emit childRemoved(child,shared_from_this(), remote);
}

std::shared_ptr<EntityBase> EntityBase::getChildById(int64_t id)
{
    for(auto & child: childs)
    {
        if(child->getId() == id)
        {
            return child;
        }
    }
    throw std::runtime_error("Child with id: " + std::to_string(id) + " not found");
}

uint64_t EntityBase::countChilds()
{
    return childs.size();
}

std::shared_ptr<EntityBase> EntityBase::getParent()
{
    return this->parent;
}

void EntityBase::setParent(std::shared_ptr<EntityBase> par)
{
    this->parent = par;
}

string EntityBase::getTopicName()
{
    if(!isSubscriber())
        return pubBase->get_topic_name();
    else
        return subBase->get_topic_name();
}


void EntityBase::IterateThroughAllProperties(std::function<void(QMetaProperty)> func)
{
    const QMetaObject* metaObj = this->metaObject();
    while(metaObj != NULL)
    {
        for (int i = metaObj->propertyOffset(); i < metaObj->propertyCount(); ++i)
        {
            func(metaObj->property(i));
        }
        metaObj = metaObj->superClass();
    }
}



}
