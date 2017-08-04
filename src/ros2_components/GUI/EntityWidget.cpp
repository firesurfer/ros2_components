#include "EntityWidget.h"
#include "ui_EntityWidget.h"

namespace ros2_components {



EntityWidget::EntityWidget(EntityBase::SharedPtr _entity, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EntityWidget)
{
    ui->setupUi(this);
    this->entity = _entity;

    model = new EntityModel(entity);
    ui->tableView->setModel(model->getModel());
}

EntityWidget::~EntityWidget()
{
    delete model;
    delete ui;
}

}
