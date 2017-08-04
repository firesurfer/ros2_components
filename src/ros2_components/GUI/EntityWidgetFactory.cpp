#include "EntityWidgetFactory.h"

namespace  ros2_components {

QHash<QString, const QMetaObject*> EntityWidgetFactory::metaWidgetObjs;

EntityWidgetFactory::EntityWidgetFactory(QObject *parent)
{

}

EntityWidget *EntityWidgetFactory::CreateInstanceFromName(string className, QGenericArgument arg1, QGenericArgument arg2)
{
    if(!metaWidgetObjs.keys().contains(QString::fromStdString(className)))
        throw std::runtime_error("Class with name: " +className+" not registered");

    const QMetaObject *meta = metaWidgetObjs[QString::fromStdString(className)];
    LOG(LogLevel::Debug) << "Class name from staticMetaObject: " << meta->className() << std::endl;
    LOG(Debug) << arg1.name() << " " << arg2.name() << std::endl;

    QObject *o = meta->newInstance(arg1,arg2);
    if(o == NULL)
        LOG(Fatal) << "Could not create instance of: " << className << std::endl;
    EntityWidget* widget = dynamic_cast<EntityWidget*>(o);

    if(widget == NULL)
        throw std::runtime_error("Could not cast QObject* to EntityWidget* - The EntityWidgetFactory is for EntityWidgets only");
    return widget;

}

EntityWidget *EntityWidgetFactory::CreateInstanceFromName(string className, QGenericArgument arg1)
{
    if(!metaWidgetObjs.keys().contains(QString::fromStdString(className)))
        throw std::runtime_error("Class with name: " +className+" not registered");

    const QMetaObject *meta = metaWidgetObjs[QString::fromStdString(className)];
    LOG(LogLevel::Debug) << "Class name from staticMetaObject: " << meta->className() << std::endl;


    QObject *o = meta->newInstance(arg1);
    if(o == NULL)
        LOG(Fatal) << "Could not create instance of: " << className << std::endl;
    EntityWidget* widget = dynamic_cast<EntityWidget*>(o);

    if(widget == NULL)
        throw std::runtime_error("Could not cast QObject* to EntityWidget* - The EntityWidgetFactory is for EntityWidgets only");
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
