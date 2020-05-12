#ifndef IGNITION_GUI_PLUGINS_PLOTTING_HH_
#define IGNITION_GUI_PLUGINS_PLOTTING_HH_

#include <ignition/gui/Application.hh>
#include <ignition/gui/qt.h>
#include <ignition/gazebo/EntityComponentManager.hh>
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include <ignition/gazebo/gui/GuiSystem.hh>
#include <string>
#include <iostream>
#include <QTimer>
#include "Transport.hh"
#include <QStandardItemModel>

#define TIMEOUT 100
using namespace std;

/// \brief Interface to Communicate with Qml, and to be a common code between the GuiPlugin and GazeboPlugin
class PlottingInterface : public QObject
{
    Q_OBJECT
private:
    /// \brief responsible for accessing the transport messages (publishing and subscribing)
    Transport* transport;
public:
    /// \brief Constructor
    PlottingInterface(Transport* _transport) : QObject()
    {
        this->transport = _transport;
    }
    /// \brief set a topic to subscribe to it (just used for testing)
    Q_INVOKABLE void setTopic(QString _topic)
    {
        this->transport->SetTopic(_topic.toStdString());
//        emit SetTopicTransport(_topic.toStdString());
    }
    /// \brief param[in] _index subscribe to the selected item in the Topics Tree which has index _index
    Q_INVOKABLE void subscribe(QModelIndex _index)
    {
        this->transport->Subscribe(_index);
//        emit SubscribeTransport(_index);
    }

    /// \brief send a signal to qml to plot a data with a value in _y and with time _x with chart num: _chart
    // modify it to use a GADNAT Object and pass it to javascript
    void emitPlotting(int _chart, float _x , QVariant _y)
    {
        emit plot(_chart , _x , _y);
    }

signals :
//    void SetTopicTransport(string _topic);
//    void SubscribeTransport(QModelIndex _index);
    void plot(int _chart,float _x , QVariant _y);
};


class GazeboPlottingPrivate ;

class GazeboPlotting : public ignition::gazebo::GuiSystem
{
    Q_OBJECT
/// \brief Constructor
public: GazeboPlotting ();
/// \brief Destructor
public: ~GazeboPlotting ();

/// \brief data_ptr holds Abstraction data of GazeboPlottingPrivate
private: unique_ptr<GazeboPlottingPrivate> data_ptr;
/// \brief get called every simulation iteration to access entities and thier components
public: void Update(const ignition::gazebo::UpdateInfo &_info,ignition::gazebo::EntityComponentManager &_ecm) override;
// x position of the car
public : float pose;
public : float time;
/// \brief send data to Qml to plot
public: void UpdateGui();

};

#endif
