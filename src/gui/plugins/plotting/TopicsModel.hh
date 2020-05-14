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
#include <QStandardItem>
#include <QStandardItemModel>
#include <QHash>
#include <QByteArray>
#include <QString>
#include <QModelIndex>
#include <bits/stdc++.h>

#define TOPIC_KEY "topic"

/// \brief Model for the TreeView in the Qml
class TopicsModel : public QStandardItemModel
{
    /// \brief Constructor
    public: TopicsModel() ;
    /// \brief add topic to the model with no children
    public: QStandardItem* AddTopic(QString _topic);

    /// \brief add _topic to the model with child _msg
    public: QStandardItem* AddTopic(QString _topic, QString _msg);

    /// \brief factory method for creating an item
    public: QStandardItem* FactoryItem(QString _topic);

    /// \brief get the topic name from its _index in the Model
    public: QString TopicName(QModelIndex _index);

    /// \brief roles of the model. to be accessed via javascript
    public: QHash<int, QByteArray> roleNames() const override;
};



