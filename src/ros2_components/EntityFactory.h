#ifndef ENTITYFACTORY_H
#define ENTITYFACTORY_H

#include <QObject>
#include <QCoreApplication>
#include <QtCore>
#include <QHash>
#include "Entity.h"

namespace ros2_components
{

//As supposed in https://stackoverflow.com/questions/19503583/qt-meta-system-call-constructor-with-parameters
#define METAOBJS_INSERT(c) (EntityFactory::metaObjs.insert(#c, &c::staticMetaObject))
class EntityFactory : public QObject
{
    Q_OBJECT
public:
    explicit EntityFactory(QObject *parent = 0);
    static QHash<QString, const QMetaObject*> metaObjs;
    static void AddQObject(QObject * obj);
    static std::shared_ptr<EntityBase> CreateInstanceFromName(std::string className, QGenericArgument arg1, QGenericArgument arg2, QGenericArgument arg3);
    static std::shared_ptr<QMetaObject> GetQMetaObject(std::string className);
signals:

public slots:
private:


};
}

#endif // ENTITYFACTORY_H
