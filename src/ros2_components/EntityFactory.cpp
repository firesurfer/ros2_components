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

#include "EntityFactory.h"
namespace ros2_components
{
QHash<QString, const QMetaObject*> EntityFactory::metaObjs;
EntityFactory::EntityFactory(QObject *parent) : QObject(parent)
{

}



void EntityFactory::AddQObject(QObject *obj)
{
    EntityFactory::metaObjs.insert(obj->metaObject()->className(), obj->metaObject());
}

std::shared_ptr<EntityBase> EntityFactory::CreateInstanceFromName(string className,QGenericArgument arg1, QGenericArgument arg2, QGenericArgument arg3)
{
    if(!metaObjs.keys().contains(QString::fromStdString(className)))
        throw std::runtime_error("Class with name: " +className+" not registered");

    const QMetaObject *meta = metaObjs[QString::fromStdString(className)];
    LOG(LogLevel::Debug) << "Class name from staticMetaObject: " << meta->className() << std::endl;

    QObject *o = meta->newInstance(arg1,arg2,arg3);
    EntityBase* ptr = dynamic_cast<EntityBase*>(o);
    if(ptr == NULL)
        throw std::runtime_error("Could not cast QObject* to EntityBase* - The EntityFactory is for Entities only");
    std::shared_ptr<EntityBase> sptr(ptr);
    LOG(LogLevel::Debug) << "Successfully created new entity: " << sptr->getName() << std::endl;
    return sptr;
}

std::shared_ptr<QMetaObject> EntityFactory::GetQMetaObject(string className)
{
    if(!metaObjs.keys().contains(QString::fromStdString(className)))
        throw std::runtime_error("Class with name: " +className+" not registered");

    const QMetaObject *meta = metaObjs[QString::fromStdString(className)];
    LOG(LogLevel::Debug) << "Class name from staticMetaObject: " << meta->className() << std::endl;
    return std::shared_ptr<QMetaObject>(const_cast<QMetaObject*>(meta));
}

bool EntityFactory::Contains(string className)
{
    QString name = QString::fromStdString(className);
    if(metaObjs.contains(name))
        return true;
    else
        return false;
}

template<typename T>
std::shared_ptr<T> EntityFactory::CreateInstanceFromType(int64_t _id, bool _subscribe, std::shared_ptr<rclcpp::node::Node> _parentNode)
{
    return std::make_shared<T>(_id,_subscribe,_parentNode);
}

template<typename T>
std::shared_ptr<T> EntityFactory::CreateInstanceFromName(string className, int64_t _id, bool _subscribe, std::shared_ptr<rclcpp::node::Node> _parentNode)
{
    QGenericArgument subscribeArg = Q_ARG(bool, _subscribe);
    QGenericArgument idArg = Q_ARG(int64_t, _id);
    QGenericArgument nodeArg  =Q_ARG(std::shared_ptr< rclcpp::node::Node >, _parentNode);

    return dynamic_pointer_cast<T>(CreateInstanceFromName(className, idArg, subscribeArg,nodeArg));
}

}
