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

#ifndef TRANSPORT_HH
#define TRANSPORT_HH
#include <QObject>
#include <QString>
#include <bits/stdc++.h>
#include <ignition/transport.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/MessageInfo.hh>
#include <ignition/transport/Publisher.hh>

#include <QVariant>
#include <QStandardItem>
#include <QStandardItemModel>

#include "TopicsModel.hh"

#define NONE "none"
#define FLOAT "float"
#define INT "int"
#define DOUBLE "double"

class TransportPrivate;

/// \brief handle Transport Topics Subscribing for one object(Chart) (will extend)
class Transport
{
    private: std::unique_ptr<TransportPrivate> data_ptr;

    /// \brief Constructor
    public: Transport() ;

    /// \brief Destructor
    public: ~Transport();

    /// \brief make the model from the available topics and messages
    public: void InitModel();

    /// \brief param[in] _index : subscribe to the topic of the selected item which has _index
    public: void Subscribe( QModelIndex _index );

    /// \brief get the Model of the topics and msgs
    public: QStandardItemModel *GetModel();

    /// \brief get the published value according to the topic's msg type, wrap it in QVariant to send to javascript
    public: QVariant GetValue();

    /// \brief callback to subscribe to topic with float msg
    public: void FloatCallback(const ignition::msgs::Float &_msg);

    /// \brief callback to subscribe to topic with int msg
    public: void Int32Callback(const ignition::msgs::Int32 &_msg);

    /// \brief callback to subscribe to topic with double msg
    public: void DoubleCallback(const ignition::msgs::Double &_msg);

    /// \brief set the topic to register
    public: void SetTopic(std::string _topic);

    /// \brief unsubscribe from the subscribed topics
    public: void Unsubscribe();

    public: void Print();

    /// \brief subscribe to the topic in data_ptr->topic
    public: void Subscribe();
};


#endif
