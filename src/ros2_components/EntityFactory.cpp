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
    std::cout << "Class name from staticMetaObject: " << meta->className() << std::endl;
    std::cout << meta->constructor(0).methodSignature().toStdString() << std::endl;

    QObject *o = meta->newInstance(arg1,arg2,arg3);
    EntityBase* ptr = dynamic_cast<EntityBase*>(o);
    if(ptr == NULL)
        throw std::runtime_error("Could not cast QObject* to EntityBase* - The EntityFactory is for Entities only");
    std::shared_ptr<EntityBase> sptr(ptr);
    return sptr;
}

std::shared_ptr<QMetaObject> EntityFactory::GetQMetaObject(string className)
{
    if(!metaObjs.keys().contains(QString::fromStdString(className)))
        throw std::runtime_error("Class with name: " +className+" not registered");

    const QMetaObject *meta = metaObjs[QString::fromStdString(className)];
    std::cout << "Class name from staticMetaObject: " << meta->className() << std::endl;
    return std::shared_ptr<QMetaObject>(const_cast<QMetaObject*>(meta));
}
}
