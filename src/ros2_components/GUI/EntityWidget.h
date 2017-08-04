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
    explicit EntityWidget(EntityBase::SharedPtr _entity, QWidget *parent = 0);
    ~EntityWidget();

private:

    Ui::EntityWidget *ui;
protected:
    EntityModel* model;
    EntityBase::SharedPtr entity;
};

}
#endif // ENTITYWIDGET_H
