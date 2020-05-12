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
using namespace std;

class TransportPrivate
{
public:
    /// \brief Node for Commincation
    ignition::transport::Node node;
    /// \brief topic to subscribe
    std::string topic;
    /// \brief Model to create it from the available topics and messages
    TopicsModel* model;

    /// \brief integer value if we subscribe to a topic with int msg
    int intValue;
    /// \brief float value if we subscribe to a topic with float msg
    float floatValue;
    /// \brief double value if we subscribe to a topic with double msg
    double doubleValue;
    /// \brief type of subscribed msg (int or float or double)
    std::string type ;
    /// \brief supported types to subscribe (int , float ,double)
    vector<string> supportedTypes;
};

/// \brief handle Transport Topics Subscribing for one object(Chart) (will extend)
class Transport
{
private:
    unique_ptr<TransportPrivate> data_ptr;

public:
    /// \brief Constructor
    Transport() : data_ptr(new TransportPrivate)
    {
        cout << "transport" << endl;
        this->data_ptr->type = NONE;
        this->data_ptr->supportedTypes.push_back("ignition.msgs.Float");
        this->data_ptr->supportedTypes.push_back("ignition.msgs.Double");
        this->data_ptr->supportedTypes.push_back("ignition.msgs.Int32");

        this->data_ptr->model = new TopicsModel();
        this->InitModel();
    }

    /// \brief Destructor
    ~Transport()
    {
        cout << "destrcutor transport" << endl;
        this->Unsubscribe();
        this->data_ptr.~unique_ptr();

    }

    /// \brief make the model from the available topics and messages
    void InitModel()
    {
        cout << "init model " << endl;
        QStandardItem* parentItem = this->data_ptr->model->invisibleRootItem();
        vector<string> allTopics ;
        this->data_ptr->node.TopicList(allTopics);

        cout << "size = " << allTopics.size() << endl;

        for(int i =0 ; i < allTopics.size(); i ++) {
            // cout << "start topic " << allTopics[i] << endl;
            std::vector<ignition::transport::MessagePublisher> infoMsgs;
            this->data_ptr->node.TopicInfo(allTopics[i],infoMsgs);
            if(infoMsgs.size() == 0) {cout <<"no info found " << endl;    continue;}


            string type = infoMsgs[0].MsgTypeName();
            //            cout << "type=" << type << endl;
            bool supported = true;
//            for(int i =0 ; i < this->supportedTypes.size(); i++)
//            {
//                if (type == this->supportedTypes[i])
//                {
//                    supported = true;
//                    //                    cout << "end topic" << endl;
//                    break;
//                }
//            }
            if(supported)
            {
                // substr 14 to remove " ignition.msgs. "
                this->data_ptr->model->AddTopic(QString::fromStdString(allTopics[i]) , QString::fromStdString(type.substr(14)) );
            }
            //            cout << "end topic" << endl;
        }

    }
    /// \brief param[in] _index : subscribe to the topic of the selected item which has _index
    void Subscribe( QModelIndex _index )
    {
        QString topic = this->data_ptr->model->TopicName(_index );
        this->SetTopic(topic.toStdString());
//        cout << "topic : " << topic.toStdString() << endl;
    }

    /// \brief get the Model of the topics and msgs
    QStandardItemModel* GetModel()
    {
        return  this->data_ptr->model;
    }

    /// \brief get the published value according to the topic's msg type, wrap it in QVariant to send to javascript
    QVariant GetValue() {
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
    void FloatCallback(const ignition::msgs::Float& _msg)
    {
        this->data_ptr->type = FLOAT;
        this->data_ptr->floatValue = _msg.data();
        cout << "float: " << this->data_ptr->floatValue << endl;
    }
    /// \brief callback to subscribe to topic with int msg
    void Int32Callback(const ignition::msgs::Int32& _msg)
    {
        this->data_ptr->type = INT;
        this->data_ptr->intValue = _msg.data();
        cout << "int: " << this->data_ptr->intValue << endl;
    }
    /// \brief callback to subscribe to topic with double msg
    void DoubleCallback(const ignition::msgs::Double& _msg)
    {
        this->data_ptr->type = DOUBLE;
        this->data_ptr->doubleValue = _msg.data();
        cout << "double: " << this->data_ptr->doubleValue << endl;
    }

    /// \brief set the topic to register
    void SetTopic(std::string _topic)
    {
        this->Unsubscribe();
        this->data_ptr->topic = _topic;

        cout <<  "set Topic : x`" << _topic << endl;

        this->Subscribe();
        this->Print();
    }

    /// \brief unsubscribe from the subscribed topics
    void Unsubscribe()
    {
        // unsubscribe from all topics (we need one topic per object for now)
        cout << "start unsubscribe" << endl;
        std::vector<string> subscribedTopics =  this->data_ptr->node.SubscribedTopics();
        for(int i =0 ; i < subscribedTopics.size(); i++)
        {
            if(this->data_ptr->node.Unsubscribe(subscribedTopics[i]))
                cout << "unsubscribed from " << subscribedTopics[i] << endl;
            else
                cout << "error unsubscribe  " << subscribedTopics[i] << endl;
        }
        cout << "end unsubscribe" << endl;
    }

    // for testing
    void Print() {

        std::vector<ignition::transport::MessagePublisher> infoMsgs;
        this->data_ptr->node.TopicInfo(this->data_ptr->topic,infoMsgs);
        if(infoMsgs.size() == 0)
        {
            cout << "print error : topic is not subscribed" << endl;
            return;
        }
    }

    /// \brief subscribe to the topic in data_ptr->topic
    void Subscribe()
    {
        // get the typeMsg of the topic to know which callback to assign
        std::vector<ignition::transport::MessagePublisher> infoMsgs;
        this->data_ptr->node.TopicInfo(this->data_ptr->topic,infoMsgs);
        // if no publishers assigned yet to that topic , return
        if(infoMsgs.size() == 0)
        {
            cout << "subscribe to" << this->data_ptr->topic << " not found" << endl;
            this->data_ptr->type = NONE;
            return;
        }
        // get any of the publishers (we assume that all publishers of the topic will be the same)
        std::string msgType = infoMsgs[0].MsgTypeName();
        cout << msgType << endl;
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
};


#endif
