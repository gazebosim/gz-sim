/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include "TopicsModel.hh"

TopicsModel :: TopicsModel() :QStandardItemModel()
{
}

//////////////////////////////////////////////////
QStandardItem* TopicsModel :: AddTopic(QString _topic)
{
    QStandardItem *item = this->FactoryItem(_topic);
    QStandardItem *parent = this->invisibleRootItem();
    parent->appendRow(item);
    return item;
}

//////////////////////////////////////////////////
QStandardItem* TopicsModel :: AddTopic(QString _topic, QString _msg)
{
    QStandardItem *msgItem = FactoryItem(_msg);
    QStandardItem *parent = this->AddTopic(_topic);
    parent->appendRow(msgItem);
    return msgItem;
}

//////////////////////////////////////////////////
QStandardItem* TopicsModel :: FactoryItem(QString _topic)
{
    QStandardItem *item = new QStandardItem(_topic);
    item->setData(QVariant(_topic), this->roleNames().key(TOPIC_KEY));
    return item;
}

//////////////////////////////////////////////////
QString TopicsModel :: TopicName(QModelIndex _index)
{
    QStandardItem *item = this->itemFromIndex(_index);
    if (!item)
        return "";

    QString topic = item->data(this->roleNames().key(TOPIC_KEY)).toString();
    return topic;
}

//////////////////////////////////////////////////
QHash<int, QByteArray> TopicsModel :: roleNames() const
{
    QHash<int, QByteArray> roles;
    roles[50] = TOPIC_KEY;
    return roles;
}




