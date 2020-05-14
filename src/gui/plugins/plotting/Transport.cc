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
#include "Transport.hh"

class TransportPrivate
{
    /// \brief Node for Commincation
    public: ignition::transport::Node node;
    /// \brief topic to subscribe
    public: std::string topic;
    /// \brief Model to create it from the available topics and messages
    public: TopicsModel *model;
    /// \brief integer value if we subscribe to a topic with int msg
    public: int intValue;
    /// \brief float value if we subscribe to a topic with float msg
    public: float floatValue;
    /// \brief double value if we subscribe to a topic with double msg
    public: double doubleValue;
    /// \brief type of subscribed msg (int or float or double)
    public: std::string type ;
    /// \brief supported types to subscribe (int , float ,double)
    public: std::vector<std::string> supportedTypes;
};

Transport :: Transport() : data_ptr(new TransportPrivate)
{
    this->data_ptr->type = NONE;
    this->data_ptr->supportedTypes.push_back("ignition.msgs.Float");
    this->data_ptr->supportedTypes.push_back("ignition.msgs.Double");
    this->data_ptr->supportedTypes.push_back("ignition.msgs.Int32");

    this->data_ptr->model = new TopicsModel();
    this->InitModel();
}

/// \brief Destructor
Transport :: ~Transport()
{
    this->Unsubscribe();
    this->data_ptr.~unique_ptr();
}

/// \brief make the model from the available topics and messages
void Transport :: InitModel()
{
    std::vector<std::string> allTopics ;
    this->data_ptr->node.TopicList(allTopics);

    for(unsigned int i =0 ; i < allTopics.size(); i ++) {
        // cout << "start topic " << allTopics[i] << endl;
        std::vector<ignition::transport::MessagePublisher> infoMsgs;
        this->data_ptr->node.TopicInfo(allTopics[i],infoMsgs);
        if(infoMsgs.size() == 0)
            continue;

        std::string type = infoMsgs[0].MsgTypeName();

        // check if the msg type is one of the supported types
        bool supported = false;
        for(unsigned int j =0 ; j < this->data_ptr->supportedTypes.size(); j++)
        {
            if (type == this->data_ptr->supportedTypes[j])
            {
                supported = true;
                break;
            }
        }
        if(supported)
        {
            // substr 14 to remove " ignition.msgs. "
            this->data_ptr->model->AddTopic(QString::fromStdString(allTopics[i]) , QString::fromStdString(type.substr(14)) );
        }
    }

}
/// \brief param[in] _index : subscribe to the topic of the selected item which has _index
void Transport :: Subscribe( QModelIndex _index )
{
    QString topic = this->data_ptr->model->TopicName(_index );
    this->SetTopic(topic.toStdString());
}

/// \brief get the Model of the topics and msgs
QStandardItemModel* Transport :: GetModel()
{
    return  this->data_ptr->model;
}

/// \brief get the published value according to the topic's msg type, wrap it in QVariant to send to javascript
QVariant Transport :: GetValue() {
    if(this->data_ptr->type == FLOAT)
        return QVariant(this->data_ptr->floatValue);
    if(this->data_ptr->type == INT)
        return QVariant(this->data_ptr->intValue);
    if(this->data_ptr->type == DOUBLE)
        return QVariant(this->data_ptr->doubleValue);
    else
        return QVariant();
}

/// \brief callback to subscribe to topic with float msg
void Transport :: FloatCallback(const ignition::msgs::Float& _msg)
{
    this->data_ptr->type = FLOAT;
    this->data_ptr->floatValue = _msg.data();
}
/// \brief callback to subscribe to topic with int msg
void Transport :: Int32Callback(const ignition::msgs::Int32& _msg)
{
    this->data_ptr->type = INT;
    this->data_ptr->intValue = _msg.data();
}
/// \brief callback to subscribe to topic with double msg
void Transport :: DoubleCallback(const ignition::msgs::Double& _msg)
{
    this->data_ptr->type = DOUBLE;
    this->data_ptr->doubleValue = _msg.data();
}

/// \brief set the topic to register
void Transport :: SetTopic(std::string _topic)
{
    this->Unsubscribe();
    this->data_ptr->topic = _topic;

    this->Subscribe();
    this->Print();
}

/// \brief unsubscribe from the subscribed topics
void Transport :: Unsubscribe()
{
    std::vector<std::string> subscribedTopics =  this->data_ptr->node.SubscribedTopics();
    for(unsigned int i =0 ; i < subscribedTopics.size(); i++)
    {
        this->data_ptr->node.Unsubscribe(subscribedTopics[i]);
    }
}

// for testing
void Transport :: Print()
{
    std::vector<ignition::transport::MessagePublisher> infoMsgs;
    this->data_ptr->node.TopicInfo(this->data_ptr->topic,infoMsgs);
    if(infoMsgs.size() == 0)
        return;
}

/// \brief subscribe to the topic in data_ptr->topic
void Transport :: Subscribe()
{
    // get the typeMsg of the topic to know which callback to assign
    std::vector<ignition::transport::MessagePublisher> infoMsgs;
    this->data_ptr->node.TopicInfo(this->data_ptr->topic,infoMsgs);
    // if no publishers assigned yet to that topic , return
    if(infoMsgs.size() == 0)
    {
        this->data_ptr->type = NONE;
        return;
    }
    // get any of the publishers (we assume that all publishers of the topic will be the same)
    std::string msgType = infoMsgs[0].MsgTypeName();
    if(msgType == "ignition.msgs.Float")
    {
        this->data_ptr->node.Subscribe(this->data_ptr->topic,&Transport::FloatCallback,this);
    }
    else if (msgType == "ignition.msgs.Double")
    {
        this->data_ptr->node.Subscribe(this->data_ptr->topic,&Transport::DoubleCallback,this);
    }
    else if (msgType == "ignition.msgs.Int32")
    {
        this->data_ptr->node.Subscribe(this->data_ptr->topic,&Transport::Int32Callback,this);
    }
}
