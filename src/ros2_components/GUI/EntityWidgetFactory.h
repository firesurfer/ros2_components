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

#ifndef ENTITYWIDGETFACTORY_H
#define ENTITYWIDGETFACTORY_H

#include <QObject>
#include <QCoreApplication>
#include <QtCore>
#include <QHash>
#include "EntityWidget.h"

#include "ros2_components_exceptions.h"

namespace  ros2_components {

//As proposed in https://stackoverflow.com/questions/19503583/qt-meta-system-call-constructor-with-parameters
#define METAWIDGETOBJS_INSERT(c) (EntityWidgetFactory::metaWidgetObjs.insert(#c, &c::staticMetaObject))


class EntityWidgetFactory
{
public:
    EntityWidgetFactory();

    static QHash<QString, const QMetaObject*> metaWidgetObjs;

    static EntityWidget *CreateInstanceFromName(std::string className, QGenericArgument arg1, QGenericArgument arg2);
    static EntityWidget *CreateInstanceFromName(std::string className, QGenericArgument arg1);

    static bool Contains(std::string className);

};
}

#endif // ENTITYWIDGETFACTORY_H
