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
    //TODO rework meta mechanism

    REFLECT(virtualEntity);
    REFLECT(className);
    REFLECT(active);
    qRegisterMetaType<int64_t>("int64_t");
    qRegisterMetaType<std::string>("std::string");

}

EntityBase::~EntityBase()
{
    std::cout << "Destroying: " << getName() << std::endl;
}

int64_t EntityBase::getId()
{
    return id;
}

string EntityBase::getName()
{
    return getClassName()  + std::to_string(id);
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
    connect(child.get(), &EntityBase::childAdded,this, &EntityBase::on_child_added,Qt::DirectConnection);
    connect(child.get(), &EntityBase::childRemoved,this, &EntityBase::on_child_removed,Qt::DirectConnection);
    emit childAdded(child,child->getParent(),0,remote);
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
    disconnect(child.get(), &EntityBase::childAdded,this, &EntityBase::on_child_added);
    emit childRemoved(child,child->getParent(),0, remote);
}

std::shared_ptr<EntityBase> EntityBase::getChildById(int64_t id)
{
    for(auto & child: childs)
    {
        if(child->getId() == id)
        {
            //Do to bug in ROS 2 this might throw an exception
            //if(!child->WasMetaInformationUpdated())
            //child->updateParameters();
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
    {
        return pubBase->get_topic_name();
    }
    else
        return subBase->get_topic_name();
}



void EntityBase::on_child_added(std::shared_ptr<EntityBase> child,std::shared_ptr<EntityBase> parent,int depth, bool remote)
{
    if(parent != NULL && child != NULL)
        LOG(LogLevel::Info) << "child"<<child->getName()<< " added to: " <<parent->getName() << " depth: " << depth << std::endl;
    emit childAdded(child,parent, depth+1,remote);
}

void EntityBase::on_child_removed(std::shared_ptr<EntityBase> child, std::shared_ptr<EntityBase> parent, int depth, bool remote)
{
    LOG(LogLevel::Info) << "child"<<child->getName()<< " removed from: " <<parent->getName() << " depth: " << depth << std::endl;
    emit childRemoved(child,parent, depth+1,remote);
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
