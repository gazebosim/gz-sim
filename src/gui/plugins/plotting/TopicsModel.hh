#include <QStandardItem>
#include <QStandardItemModel>
#include <QHash>
#include <QByteArray>
#include <QString>
#include <QModelIndex>
#include <bits/stdc++.h>
using namespace std;

#define TOPIC_KEY "topic"

/// \brief Model for the TreeView in the Qml
class TopicsModel : public QStandardItemModel {
public:
    /// \brief Constructor
    TopicsModel() :QStandardItemModel()
    {
        cout << "constructor Topics Model" << endl;
    }
    /// \brief add topic to the model with no children
    QStandardItem* AddTopic(QString _topic)
    {
        QStandardItem* item = this->FactoryItem(_topic);
        QStandardItem* parent = this->invisibleRootItem();
        parent->appendRow(item);

        cout <<"added " << _topic.toStdString() << endl;
        return item;
    }
    /// \brief add _topic to the model with child _msg
    QStandardItem* AddTopic(QString _topic, QString _msg)
    {
        QStandardItem* msgItem = FactoryItem(_msg);
        QStandardItem* parent = this->AddTopic(_topic);
        parent->appendRow(msgItem);
        cout << parent->text().toStdString() << "," <<msgItem->text().toStdString() << endl;
        return msgItem;
    }

    /// \brief factory method for creating an item
    QStandardItem* FactoryItem(QString _topic)
    {
        QStandardItem* item = new QStandardItem(_topic);
        item->setData(QVariant(_topic),this->roleNames().key(TOPIC_KEY));
        return item;
    }

    /// \brief get the topic name from its _index in the Model
    QString TopicName(QModelIndex _index)
    {
        QStandardItem* item = this->itemFromIndex(_index);
        if(! item)
            return "";

        QString topic = item->data(this->roleNames().key(TOPIC_KEY)).toString();
        return topic;
    }

    /// \brief roles of the model. to be accessed via javascript
    QHash<int, QByteArray> roleNames() const override
    {
        QHash<int,QByteArray> roles;
        roles[50] = TOPIC_KEY;
        return roles;
    }

};



