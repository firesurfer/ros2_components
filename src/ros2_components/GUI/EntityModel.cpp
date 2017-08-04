#include "EntityModel.h"

namespace ros2_components {


EntityModel::EntityModel(EntityBase::SharedPtr entity)
{
    //TODO which size?
    model = new QStandardItemModel(0,0);
    QStandardItem* idItem = new QStandardItem(QString::number(entity->getId()));
    QStandardItem* idLabelItem = new QStandardItem("Id");
    model->insertRow(model->rowCount(), idItem);
    model->setVerticalHeaderItem(model->rowCount()-1, idLabelItem);

    QStandardItem* nameItem = new QStandardItem(QString::fromStdString(entity->getName()));
    QStandardItem* nameLabelItem = new QStandardItem("Name");
    model->insertRow(model->rowCount(), nameItem);
    model->setVerticalHeaderItem(model->rowCount()-1, nameLabelItem);

    QStandardItem* topicItem = new QStandardItem(QString::fromStdString(entity->getTopicName()));
    QStandardItem* topicLabelItem = new QStandardItem("Topic");
    model->insertRow(model->rowCount(), topicItem);
    model->setVerticalHeaderItem(model->rowCount()-1, topicLabelItem);

    QStandardItem* classItem = new QStandardItem(QString::fromStdString(entity->getClassName()));
    QStandardItem* classLabelItem = new QStandardItem("Classname");
    model->insertRow(model->rowCount(), classItem);
    model->setVerticalHeaderItem(model->rowCount()-1, classLabelItem);

    QStandardItem* descItem = new QStandardItem(QString::fromStdString((entity->getDescription())));
    QStandardItem* desclabeItem = new QStandardItem("Description");
    model->insertRow(model->rowCount(), descItem);
    model->setVerticalHeaderItem(model->rowCount()-1, desclabeItem);

}

EntityModel::~EntityModel()
{
    delete model;
}

QStandardItemModel *EntityModel::getModel() const
{
    return model;
}

}
