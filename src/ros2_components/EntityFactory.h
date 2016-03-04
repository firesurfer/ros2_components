#pragma once

#include <QObject>
#include <QCoreApplication>
#include <QtCore>
#include <QHash>
#include "Entity.h"

namespace ros2_components
{

//As supposed in https://stackoverflow.com/questions/19503583/qt-meta-system-call-constructor-with-parameters
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
signals:

public slots:
private:


};
}


