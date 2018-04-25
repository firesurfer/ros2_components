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

#include "EntityWidget.h"
#include "ui_EntityWidget.h"

namespace ros2_components {



EntityWidget::EntityWidget(EntityBase::SharedPtr _entity, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EntityWidget)
{
    ui->setupUi(this);
    this->entity = _entity;

    ui->label->setText(QString::fromStdString(entity->getName()));
    model = new EntityModel(entity);
    ui->tableView->setModel(model->getModel());
}


EntityWidget::~EntityWidget()
{
    delete model;
    delete ui;
}

}
