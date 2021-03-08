#include "SphericalCoordinates.hh"
#include <ignition/math/SphericalCoordinates.hh>
namespace tethys_spherical_coords 
{

// Parameters for EARTH_WGS84 model
// wikipedia: World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84



class SphericalCoordinatePrivateData
{
  public: std::mutex mtx;

  public: ignition::gazebo::Entity _linkEntity;
  
  public: ignition::transport::Node node;

  public: ignition::math::SphericalCoordinates coords;

  public: ignition::transport::Node::Publisher latlonPub;
};

SphericalCoordinatePlugin::SphericalCoordinatePlugin()
{
    _data = std::make_unique<SphericalCoordinatePrivateData>();
}

SphericalCoordinatePlugin::~SphericalCoordinatePlugin()
{

}

void SphericalCoordinatePlugin::Configure(
  const ignition::gazebo::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  ignition::gazebo::EntityComponentManager &_ecm,
  ignition::gazebo::EventManager &/*_eventMgr*/)
{
  // Get Joint name
  if (!_sdf->HasElement("link_name")) 
  {
    ignerr << "No joint to track \n";
    return;
  }
  auto link_name = _sdf->Get<std::string>("link_name");


  if(_sdf->HasElement("start_lattitude"))
  {
    auto start_lat = _sdf->Get<double>("start_lattitude");
    ignition::math::Angle angle;
    angle.Degree(start_lat);
    _data->coords.SetLatitudeReference(angle); 
  }

  if(_sdf->HasElement("start_longitude"))
  {
    auto start_lon = _sdf->Get<double>("start_longitude");
    ignition::math::Angle angle;
    angle.Degree(start_lon);
    _data->coords.SetLongitudeReference(angle); 
  }
  _data->coords.UpdateTransformationMatrix();
}

void SphericalCoordinatePlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  ignition::gazebo::Link link(_data->_linkEntity);

  auto pose = worldPose(_data->_linkEntity, _ecm);
  
  auto latlong = _data->coords.SphericalFromLocalPosition(pose.Pos());


}
}

IGNITION_ADD_PLUGIN(
  tethys_spherical_coords::SphericalCoordinatePlugin,
  ignition::gazebo::System,
  tethys_spherical_coords::SphericalCoordinatePlugin::ISystemConfigure,
  tethys_spherical_coords::SphericalCoordinatePlugin::ISystemPreUpdate)