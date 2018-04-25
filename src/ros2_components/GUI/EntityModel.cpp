/*
 * Copyright 2018 <Lennart Nachtigall> <firesurfer127@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

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



    auto func = [&](Element* elem )
    {
        QStandardItem* reflLabelItem = new QStandardItem("Auto prop: " + QString::fromStdString(elem->getKey()));
        if(elem->getType() == "std::string" || elem->getType() == "string")
        {
            std::string val = ((SpecificElement<std::string>*)elem)->to_string();
            QStandardItem* reflItem = new QStandardItem(QString::fromStdString(val));

            model->insertRow(model->rowCount(), reflItem);
            model->setVerticalHeaderItem(model->rowCount()-1, reflLabelItem);
        }
        else if(elem->getType() == "int64_t")
        {
            std::string key = "";
            std::string type ="";
            int64_t val = *(int64_t*)((SpecificElement<int64_t>*)elem)->getBytes(key,type).data();
            QStandardItem* reflItem = new QStandardItem(QString::number(val));

            model->insertRow(model->rowCount(), reflItem);
            model->setVerticalHeaderItem(model->rowCount()-1, reflLabelItem);

        }
        else if(elem->getType() == "double")
        {

            std::string key = "";
            std::string type ="";
            double val = *(double*)((SpecificElement<double>*)elem)->getBytes(key,type).data();
            QStandardItem* reflItem = new QStandardItem(QString::number(val));

            model->insertRow(model->rowCount(), reflItem);
            model->setVerticalHeaderItem(model->rowCount()-1, reflLabelItem);
        }
        else if(elem->getType() == "bool")
        {
            std::string key = "";
            std::string type ="";
            bool val = *(bool*)((SpecificElement<bool>*)elem)->getBytes(key,type).data();
            QStandardItem* reflItem = new QStandardItem(QString::fromStdString(val ? "true" : "false"));

            model->insertRow(model->rowCount(), reflItem);
            model->setVerticalHeaderItem(model->rowCount()-1, reflLabelItem);

        }
        else
        {
            //We could not do anything with it
            delete reflLabelItem;
        }

    };
    entity->iterateThroughAllProperties(func);

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
