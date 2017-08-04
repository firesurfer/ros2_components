#ifndef ENTITYWIDGET_H
#define ENTITYWIDGET_H

#include <QWidget>
#include "EntityBase.h"
#include "EntityModel.h"

namespace Ui {
class EntityWidget;
}

namespace  ros2_components {


class EntityWidget : public QWidget
{
    Q_OBJECT

public:
    Q_INVOKABLE EntityWidget(EntityBase::SharedPtr _entity, QWidget *parent = 0);

    ~EntityWidget();

private:


protected:
    EntityModel* model;
    EntityBase::SharedPtr entity;
     Ui::EntityWidget *ui;
};

}
#endif // ENTITYWIDGET_H
