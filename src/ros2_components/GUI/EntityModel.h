#ifndef ENTITYMODEL_H
#define ENTITYMODEL_H


#include <QStandardItemModel>
#include "EntityBase.h"

namespace  ros2_components {

class EntityModel
{
public:
    EntityModel(EntityBase::SharedPtr entity);
    virtual ~EntityModel();
    QStandardItemModel *getModel() const;

private:
    QStandardItemModel* model;

};
}

#endif // ENTITYMODEL_H
