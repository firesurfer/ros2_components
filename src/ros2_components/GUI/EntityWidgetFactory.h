#ifndef ENTITYWIDGETFACTORY_H
#define ENTITYWIDGETFACTORY_H

#include <QObject>
#include <QCoreApplication>
#include <QtCore>
#include <QHash>
#include "EntityWidget.h"

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
