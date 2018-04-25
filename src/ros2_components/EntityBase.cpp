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


#include "EntityBase.h"
namespace ros2_components {

EntityBase::EntityBase(int64_t _id, bool _subscribe,  NodeContainer::SharedPtr _nodeContainer, string _className):
    id{_id},
    subscriber{_subscribe},
    nodeContainer{_nodeContainer},
    className{_className}
{

    this->className = _className;
    this->active = true;
    this->virtualEntity = false;
    this->name = getClassName()  + std::to_string(id);


    REFLECT(className);
    REFLECT(active);
    REFLECT(description)
    qRegisterMetaType<int64_t>("int64_t");
    qRegisterMetaType<std::string>("std::string");


}

EntityBase::EntityBase(int64_t _id, bool _subscribe,  NodeContainer::SharedPtr _nodeContainer, string _className, string _componentName):EntityBase(_id,_subscribe, _nodeContainer,_className)
{
    this->name = _componentName + std::to_string(_id);
}

EntityBase::~EntityBase()
{
    for (auto e : internalmap)
    {
        delete e;
    }

    if (!isVirtual())
    {
        ComponentInfo info;
        info.id = getId();
        std::shared_ptr<EntityBase> parent_ptr = parent.lock();
        if (parent_ptr)
        {
            info.parentId = parent_ptr->getId();
            info.parentType = parent_ptr->getClassName();
        }
        info.name = getName();
        info.nodename = nodeContainer->getRosNode()->get_name();
        info.type = getClassName();
        emit entityDeleted(info);
    }
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

rclcpp::Node::SharedPtr EntityBase::getParentNode()
{
    return parentNode;
}

NodeContainer::SharedPtr EntityBase::getNodeContainer()
{
    return nodeContainer;
}

void EntityBase::addChild(std::shared_ptr<EntityBase> child, bool remote)
{
    LOG(LogLevel::Debug) << "addChild called with: " << child->getName() << " from: " << getName()<< std::endl;

    std::function<bool (std::shared_ptr<EntityBase>, EntityBase*)> has_child =
            [&] (std::shared_ptr<EntityBase> entity, EntityBase* child)
    {
        if (entity.get() == child)
        {
            return true;
        }
        for (auto e : entity->childs)
        {
            if (has_child(e, child))
            {
                return true;
            }
        }
        return false;
    };
    if (has_child(child, this))
    {
        throw EntityCircleException("Adding Child " + child->getName() + " to " + getName()
                                    + " creates cycle");
    }

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
        throw MissingChildException("Can't remove given child - child not found!");
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
    throw MissingChildException("Child with id: " + std::to_string(id) + " not found");
}

uint64_t EntityBase::countChilds()
{
    return childs.size();
}

std::shared_ptr<EntityBase> EntityBase::getParent()
{
    return this->parent.lock();
}

void EntityBase::setParent(std::shared_ptr<EntityBase> par)
{
    this->parent = par;
}

string EntityBase::getTopicName()
{
    if(subBase || pubBase)
    {
        if(!isSubscriber())
            return pubBase->get_topic_name();
        else
            return subBase->get_topic_name();
    }
    else
        return name;
}


void EntityBase::iterateThroughAllProperties(std::function<void(Element *)> func)
{
    for(Element* elem: this->internalmap)
    {
        if(func != NULL)
            func(elem);
    }
}

void EntityBase::iterateThroughAllChilds(std::function<void (EntityBase::SharedPtr)> func)
{
    std::function<void(EntityBase::SharedPtr)> rec_func = [&](EntityBase::SharedPtr base){
        for(auto &  child : base->getAllChilds())
        {
            func(child);
            rec_func(child);

        }

    };
    for(auto & seg: getAllChilds())
    {
        func(seg);
        rec_func(seg);
    }
}



}
