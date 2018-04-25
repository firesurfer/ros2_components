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

#include "EntityWidgetFactory.h"

namespace  ros2_components {

QHash<QString, const QMetaObject*> EntityWidgetFactory::metaWidgetObjs;

EntityWidgetFactory::EntityWidgetFactory()
{

}

EntityWidget *EntityWidgetFactory::CreateInstanceFromName(string className, QGenericArgument arg1, QGenericArgument arg2)
{
    if(!metaWidgetObjs.keys().contains(QString::fromStdString(className)))
        throw EntityWidgetNotRegisteredException("Class with name: " +className+" not registered");

    const QMetaObject *meta = metaWidgetObjs[QString::fromStdString(className)];
    LOG(LogLevel::Debug) << "Class name from staticMetaObject: " << meta->className() << std::endl;
    LOG(Debug) << arg1.name() << " " << arg2.name() << std::endl;

    QObject *o = meta->newInstance(arg1,arg2);
    if(o == NULL)
        LOG(Fatal) << "Could not create instance of: " << className << std::endl;
    EntityWidget* widget = dynamic_cast<EntityWidget*>(o);

    if(widget == NULL)
        throw EntityWidgetCastException("Could not cast QObject* to EntityWidget* - The EntityWidgetFactory is for EntityWidgets only");
    return widget;

}

EntityWidget *EntityWidgetFactory::CreateInstanceFromName(string className, QGenericArgument arg1)
{
    if(!metaWidgetObjs.keys().contains(QString::fromStdString(className)))
        throw EntityWidgetNotRegisteredException("Class with name: " +className+" not registered");

    const QMetaObject *meta = metaWidgetObjs[QString::fromStdString(className)];
    LOG(LogLevel::Debug) << "Class name from staticMetaObject: " << meta->className() << std::endl;


    QObject *o = meta->newInstance(arg1);
    if(o == NULL)
        LOG(Fatal) << "Could not create instance of: " << className << std::endl;
    EntityWidget* widget = dynamic_cast<EntityWidget*>(o);

    if(widget == NULL)
        throw EntityWidgetCastException("Could not cast QObject* to EntityWidget* - The EntityWidgetFactory is for EntityWidgets only");
    return widget;
}

bool EntityWidgetFactory::Contains(string className)
{
    QString name = QString::fromStdString(className);
    if(metaWidgetObjs.contains(name))
        return true;
    else
        return false;
}
}
