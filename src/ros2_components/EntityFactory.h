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

#pragma once

#include <QObject>
#include <QCoreApplication>
#include <QtCore>
#include <QHash>
#include "Entity.h"

namespace ros2_components
{

//As proposed in https://stackoverflow.com/questions/19503583/qt-meta-system-call-constructor-with-parameters
#define METAOBJS_INSERT(c) (EntityFactory::metaObjs.insert(#c, &c::staticMetaObject))

/**
 * @brief The EntityFactory class
 * This is a factory that helps you creating instances of classes that inherit from Entity<MessageType> and that are registered via METAOBJS_INSERT
 * simply call: METAOBJS_INSERT(MyType) for all Types you want to register to the system.
 * Your classes may only have 3 constructor arguments
 */
class EntityFactory : public QObject
{
    Q_OBJECT
public:
    explicit EntityFactory(QObject *parent = 0);
    /**
     * @brief metaObjs
     */
    static QHash<QString, const QMetaObject*> metaObjs;
    /**
     * @brief AddQObject
     * @param obj
     */
    static void AddQObject(QObject * obj);
    /**
     * @brief CreateInstanceFromName
     * @param className
     * @param arg1
     * @param arg2
     * @param arg3
     * @return
     */
    static std::shared_ptr<EntityBase> CreateInstanceFromName(std::string className, QGenericArgument arg1, QGenericArgument arg2, QGenericArgument arg3);
    /**
     * @brief GetQMetaObject
     * @param className
     * @return
     */
    static std::shared_ptr<QMetaObject> GetQMetaObject(std::string className);
    /**
     * @brief Contains
     * @param className
     * @return True if the given className is available
     */
    static bool Contains(std::string className);

signals:

public slots:
private:


};
}


