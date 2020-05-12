#include <ignition/plugin/Register.hh>

#include "GazeboPlotting.hh"
using namespace ignition::gui;
using namespace ignition::gazebo;

class GazeboPlottingPrivate
{
public:
    /// \brief Transport Object for Subscribing and Publishing Topics with Msgs
    Transport* transport;
    /// \brief Interface to communicate with Qml
    PlottingInterface* plottingIface;

};


GazeboPlotting ::GazeboPlotting ()  : GuiSystem() , data_ptr(new GazeboPlottingPrivate)
{
    this->pose=0;
    this->time=0;
    this->data_ptr->transport = new Transport();
    this->data_ptr->plottingIface = new PlottingInterface(this->data_ptr->transport);

    ignition::gui::App()->Engine()->rootContext()->setContextProperty(
        "TopicsModel", this->data_ptr->transport->GetModel());

    ignition::gui::App()->Engine()->rootContext()->setContextProperty(
                "PlottingIface",this->data_ptr->plottingIface);
}

GazeboPlotting ::~GazeboPlotting ()
{
    delete this->data_ptr->transport;
    delete this->data_ptr->plottingIface;
    this->data_ptr.~unique_ptr();
}

void GazeboPlotting ::Update(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &_ecm)
{
    if(_info.paused)
        return;

    int realTime = _info.realTime.count()/1000000000;

    // get the blue car entity by its name
    auto CarEntity = _ecm.EntityByComponents(components::Name("vehicle_blue"));
    if(!CarEntity)
        return;

//    // get the position component of the car entity
    auto _pose = _ecm.Component<components::Pose>(CarEntity);
    if(!_pose)
        return;

    this->pose = _pose->Data().Pos().X();

    if (realTime  <= this->time )
        return;

    this->time = realTime;
//    cout <<"ray2 time= " <<  this->time << endl;
    this->UpdateGui();
}


void GazeboPlotting ::UpdateGui()
{
    this->data_ptr->plottingIface->emitPlotting(0,this->time,this->pose);
    this->data_ptr->plottingIface->emitPlotting(1,this->time,this->data_ptr->transport->GetValue());

    cout << "time:" << this->time << "  " << this->data_ptr->transport->GetValue().toFloat() << endl;
}

// Register this plugin
IGNITION_ADD_PLUGIN(GazeboPlotting ,
                    ignition::gazebo::GuiSystem,
                    ignition::gui::Plugin)
