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
        throw std::runtime_error("Class not registered");

    const QMetaObject *meta = metaObjs["MyQObject"];
        std::cout << "Class name from staticMetaObject: " << meta->className() << std::endl;
        QObject *o = meta->newInstance(); // add constructor arguments as needed
}
}
