#ifndef ENTITYWIDGETFACTORY_H
#define ENTITYWIDGETFACTORY_H

#include <QObject>
#include <QCoreApplication>
#include <QtCore>
#include <QHash>
#include "EntityWidget.h"

namespace  ros2_components {

//As proposed in https://stackoverflow.com/questions/19503583/qt-meta-system-call-constructor-with-parameters
#define METAOWIDGETBJS_INSERT(c) (EntityWidgetFactory::metaWidgetObjs.insert(#c, &c::staticMetaObject))


class EntityWidgetFactory:public QObject
{
    Q_OBJECT
public:
    EntityWidgetFactory(QObject * parent=0);

    static QHash<QString, const QMetaObject*> metaWidgetObjs;

    static EntityWidget *CreateInstanceFromName(std::string className, QGenericArgument arg1, QGenericArgument arg2);
};
}

#endif // ENTITYWIDGETFACTORY_H
