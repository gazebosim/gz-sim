/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <ignition/msgs/atmosphere.pb.h>
#include <ignition/msgs/boxgeom.pb.h>
#include <ignition/msgs/cylindergeom.pb.h>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/geometry.pb.h>
#include <ignition/msgs/gui.pb.h>
#include <ignition/msgs/imu_sensor.pb.h>
#include <ignition/msgs/lidar_sensor.pb.h>
#include <ignition/msgs/actor.pb.h>
#include <ignition/msgs/light.pb.h>
#include <ignition/msgs/material.pb.h>
#include <ignition/msgs/planegeom.pb.h>
#include <ignition/msgs/plugin.pb.h>
#include <ignition/msgs/spheregeom.pb.h>
#include <ignition/msgs/Utility.hh>

#include <ignition/math/Angle.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/math/Temperature.hh>

#include <ignition/common/Console.hh>

#include <sdf/Actor.hh>
#include <sdf/Atmosphere.hh>
#include <sdf/AirPressure.hh>
#include <sdf/Altimeter.hh>
#include <sdf/Box.hh>
#include <sdf/Camera.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Geometry.hh>
#include <sdf/Gui.hh>
#include <sdf/Imu.hh>
#include <sdf/Lidar.hh>
#include <sdf/Light.hh>
#include <sdf/Magnetometer.hh>
#include <sdf/Material.hh>
#include <sdf/Mesh.hh>
#include <sdf/Pbr.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

#include <string>

#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Util.hh"

using namespace ignition;

//////////////////////////////////////////////////
template<>
msgs::Entity_Type ignition::gazebo::convert(const std::string &_in)
{
  msgs::Entity_Type out = msgs::Entity_Type_NONE;

  if (_in == "light") {
    return msgs::Entity_Type_LIGHT;
  }
  else if (_in == "model")
  {
    return msgs::Entity_Type_MODEL;
  }
  else if (_in == "link")
  {
    return msgs::Entity_Type_LINK;
  }
  else if (_in == "visual")
  {
    return msgs::Entity_Type_VISUAL;
  }
  else if (_in == "collision")
  {
    return msgs::Entity_Type_COLLISION;
  }
  else if (_in == "sensor")
  {
    return msgs::Entity_Type_SENSOR;
  }
  else if (_in == "joint")
  {
    return msgs::Entity_Type_JOINT;
  }

  return out;
}

//////////////////////////////////////////////////
template<>
math::Pose3d ignition::gazebo::convert(const msgs::Pose &_in)
{
  math::Pose3d out(_in.position().x(),
                   _in.position().y(),
                   _in.position().z(),
                   _in.orientation().w(),
                   _in.orientation().x(),
                   _in.orientation().y(),
                   _in.orientation().z());

  return out;
}

//////////////////////////////////////////////////
template<>
msgs::Collision ignition::gazebo::convert(const sdf::Collision &_in)
{
  msgs::Collision out;
  out.set_name(_in.Name());
  msgs::Set(out.mutable_pose(), _in.RawPose());
  out.mutable_geometry()->CopyFrom(convert<msgs::Geometry>(*_in.Geom()));

  return out;
}

//////////////////////////////////////////////////
template<>
sdf::Collision ignition::gazebo::convert(const msgs::Collision &_in)
{
  sdf::Collision out;
  out.SetName(_in.name());
  out.SetRawPose(msgs::Convert(_in.pose()));
  out.SetGeom(convert<sdf::Geometry>(_in.geometry()));
  return out;
}

//////////////////////////////////////////////////
template<>
msgs::Geometry ignition::gazebo::convert(const sdf::Geometry &_in)
{
  msgs::Geometry out;
  if (_in.Type() == sdf::GeometryType::BOX && _in.BoxShape())
  {
    out.set_type(msgs::Geometry::BOX);
    msgs::Set(out.mutable_box()->mutable_size(), _in.BoxShape()->Size());
  }
  else if (_in.Type() == sdf::GeometryType::CYLINDER && _in.CylinderShape())
  {
    out.set_type(msgs::Geometry::CYLINDER);
    out.mutable_cylinder()->set_radius(_in.CylinderShape()->Radius());
    out.mutable_cylinder()->set_length(_in.CylinderShape()->Length());
  }
  else if (_in.Type() == sdf::GeometryType::PLANE && _in.PlaneShape())
  {
    out.set_type(msgs::Geometry::PLANE);
    msgs::Set(out.mutable_plane()->mutable_normal(),
              _in.PlaneShape()->Normal());
    msgs::Set(out.mutable_plane()->mutable_size(),
              _in.PlaneShape()->Size());
  }
  else if (_in.Type() == sdf::GeometryType::SPHERE && _in.SphereShape())
  {
    out.set_type(msgs::Geometry::SPHERE);
    out.mutable_sphere()->set_radius(_in.SphereShape()->Radius());
  }
  else if (_in.Type() == sdf::GeometryType::MESH && _in.MeshShape())
  {
    auto meshSdf = _in.MeshShape();

    out.set_type(msgs::Geometry::MESH);
    auto meshMsg = out.mutable_mesh();

    msgs::Set(meshMsg->mutable_scale(), meshSdf->Scale());
    meshMsg->set_filename(asFullPath(meshSdf->Uri(), meshSdf->FilePath()));
    meshMsg->set_submesh(meshSdf->Submesh());
    meshMsg->set_center_submesh(meshSdf->CenterSubmesh());
  }
  else
  {
    ignerr << "Geometry type [" << static_cast<int>(_in.Type())
           << "] not supported" << std::endl;
  }
  return out;
}

//////////////////////////////////////////////////
template<>
sdf::Geometry ignition::gazebo::convert(const msgs::Geometry &_in)
{
  sdf::Geometry out;
  if (_in.type() == msgs::Geometry::BOX && _in.has_box())
  {
    out.SetType(sdf::GeometryType::BOX);

    sdf::Box boxShape;
    boxShape.SetSize(msgs::Convert(_in.box().size()));

    out.SetBoxShape(boxShape);
  }
  else if (_in.type() == msgs::Geometry::CYLINDER && _in.has_cylinder())
  {
    out.SetType(sdf::GeometryType::CYLINDER);

    sdf::Cylinder cylinderShape;
    cylinderShape.SetRadius(_in.cylinder().radius());
    cylinderShape.SetLength(_in.cylinder().length());

    out.SetCylinderShape(cylinderShape);
  }
  else if (_in.type() == msgs::Geometry::PLANE && _in.has_plane())
  {
    out.SetType(sdf::GeometryType::PLANE);

    sdf::Plane planeShape;
    planeShape.SetNormal(msgs::Convert(_in.plane().normal()));
    planeShape.SetSize(msgs::Convert(_in.plane().size()));

    out.SetPlaneShape(planeShape);
  }
  else if (_in.type() == msgs::Geometry::SPHERE && _in.has_sphere())
  {
    out.SetType(sdf::GeometryType::SPHERE);

    sdf::Sphere sphereShape;
    sphereShape.SetRadius(_in.sphere().radius());

    out.SetSphereShape(sphereShape);
  }
  else if (_in.type() == msgs::Geometry::MESH && _in.has_mesh())
  {
    out.SetType(sdf::GeometryType::MESH);

    sdf::Mesh meshShape;
    meshShape.SetScale(msgs::Convert(_in.mesh().scale()));
    meshShape.SetUri(_in.mesh().filename());
    meshShape.SetSubmesh(_in.mesh().submesh());
    meshShape.SetCenterSubmesh(_in.mesh().center_submesh());

    out.SetMeshShape(meshShape);
  }
  else
  {
    ignerr << "Geometry type [" << static_cast<int>(_in.type())
           << "] not supported" << std::endl;
  }
  return out;
}

//////////////////////////////////////////////////
template<>
msgs::Material ignition::gazebo::convert(const sdf::Material &_in)
{
  msgs::Material out;
  msgs::Set(out.mutable_ambient(), _in.Ambient());
  msgs::Set(out.mutable_diffuse(), _in.Diffuse());
  msgs::Set(out.mutable_specular(), _in.Specular());
  msgs::Set(out.mutable_emissive(), _in.Emissive());
  out.set_lighting(_in.Lighting());

  sdf::Pbr *pbr = _in.PbrMaterial();
  if (pbr)
  {
    msgs::Material::PBR *pbrMsg = out.mutable_pbr();
    sdf::PbrWorkflow *workflow = pbr->Workflow(sdf::PbrWorkflowType::METAL);
    if (workflow)
      pbrMsg->set_type(msgs::Material_PBR_WorkflowType_METAL);
    else
    {
      workflow = pbr->Workflow(sdf::PbrWorkflowType::SPECULAR);
      if (workflow)
        pbrMsg->set_type(msgs::Material_PBR_WorkflowType_SPECULAR);
    }
    if (workflow)
    {
      pbrMsg->set_metalness(workflow->Metalness());
      pbrMsg->set_metalness_map(workflow->MetalnessMap());
      pbrMsg->set_roughness(workflow->Roughness());
      pbrMsg->set_roughness_map(workflow->RoughnessMap());
      pbrMsg->set_glossiness(workflow->Glossiness());
      pbrMsg->set_glossiness_map(workflow->GlossinessMap());
      pbrMsg->set_specular_map(workflow->SpecularMap());
      pbrMsg->set_albedo_map(workflow->AlbedoMap());
      pbrMsg->set_normal_map(workflow->NormalMap());
      pbrMsg->set_ambient_occlusion_map(workflow->AmbientOcclusionMap());
      pbrMsg->set_environment_map(workflow->EnvironmentMap());
      pbrMsg->set_emissive_map(workflow->EmissiveMap());
    }
  }
  return out;
}

//////////////////////////////////////////////////
template<>
sdf::Material ignition::gazebo::convert(const msgs::Material &_in)
{
  sdf::Material out;
  out.SetAmbient(msgs::Convert(_in.ambient()));
  out.SetDiffuse(msgs::Convert(_in.diffuse()));
  out.SetSpecular(msgs::Convert(_in.specular()));
  out.SetEmissive(msgs::Convert(_in.emissive()));
  out.SetLighting(_in.lighting());

  if (_in.has_pbr())
  {
    const auto &pbrMsg = _in.pbr();
    sdf::Pbr pbr;
    sdf::PbrWorkflow workflow;
    if (pbrMsg.type() == msgs::Material_PBR_WorkflowType_METAL)
      workflow.SetType(sdf::PbrWorkflowType::METAL);
    else if (pbrMsg.type() == msgs::Material_PBR_WorkflowType_SPECULAR)
      workflow.SetType(sdf::PbrWorkflowType::SPECULAR);
    workflow.SetAlbedoMap(pbrMsg.albedo_map());
    workflow.SetNormalMap(pbrMsg.normal_map());
    workflow.SetMetalness(pbrMsg.metalness());
    workflow.SetMetalnessMap(pbrMsg.metalness_map());
    workflow.SetRoughness(pbrMsg.roughness());
    workflow.SetRoughnessMap(pbrMsg.roughness_map());
    workflow.SetGlossiness(pbrMsg.glossiness());
    workflow.SetGlossinessMap(pbrMsg.glossiness_map());
    workflow.SetSpecularMap(pbrMsg.specular_map());
    workflow.SetEnvironmentMap(pbrMsg.environment_map());
    workflow.SetAmbientOcclusionMap(pbrMsg.ambient_occlusion_map());
    workflow.SetEmissiveMap(pbrMsg.emissive_map());
    pbr.SetWorkflow(workflow.Type(), workflow);
    out.SetPbrMaterial(pbr);
  }
  return out;
}

//////////////////////////////////////////////////
template<>
msgs::Actor ignition::gazebo::convert(const sdf::Actor &_in)
{
  msgs::Actor out;
  out.mutable_entity()->set_name(_in.Name());
  msgs::Set(out.mutable_pose(), _in.RawPose());
  out.set_skin_filename(asFullPath(_in.SkinFilename(), _in.FilePath()));
  out.set_skin_scale(_in.SkinScale());
  for (unsigned int i = 0; i < _in.AnimationCount(); ++i)
  {
    auto newAnim = out.add_animations();
    auto anim = _in.AnimationByIndex(i);
    newAnim->set_name(anim->Name());
    newAnim->set_filename(asFullPath(anim->Filename(), anim->FilePath()));
    newAnim->set_scale(anim->Scale());
    newAnim->set_interpolate_x(anim->InterpolateX());
  }
  out.set_script_loop(_in.ScriptLoop());
  out.set_script_delay_start(_in.ScriptDelayStart());
  out.set_script_auto_start(_in.ScriptAutoStart());
  for (unsigned int i = 0; i < _in.TrajectoryCount(); ++i)
  {
    auto newTraj = out.add_trajectories();
    auto traj = _in.TrajectoryByIndex(i);
    newTraj->set_id(traj->Id());
    newTraj->set_type(traj->Type());
    newTraj->set_tension(traj->Tension());
    for (unsigned int j = 0; j < traj->WaypointCount(); ++j)
    {
      auto newPoint = newTraj->add_waypoints();
      auto point = traj->WaypointByIndex(j);
      newPoint->set_time(point->Time());
      msgs::Set(newPoint->mutable_pose(), point->Pose());
    }
  }
  return out;
}

//////////////////////////////////////////////////
template<>
sdf::Actor ignition::gazebo::convert(const msgs::Actor &_in)
{
  sdf::Actor out;
  out.SetName(_in.entity().name());
  out.SetRawPose(msgs::Convert(_in.pose()));
  out.SetSkinFilename(_in.skin_filename());
  out.SetSkinScale(_in.skin_scale());
  for (int i = 0; i < _in.animations_size(); ++i)
  {
    const auto &anim = _in.animations(i);
    auto newAnim = new sdf::Animation();
    newAnim->SetName(anim.name());
    newAnim->SetFilename(anim.filename());
    newAnim->SetScale(anim.scale());
    newAnim->SetInterpolateX(anim.interpolate_x());
    out.AddAnimation(*newAnim);
  }
  out.SetScriptLoop(_in.script_loop());
  out.SetScriptDelayStart(_in.script_delay_start());
  out.SetScriptAutoStart(_in.script_auto_start());
  for (int i = 0; i < _in.trajectories_size(); ++i)
  {
    const auto &traj = _in.trajectories(i);
    auto newTraj = new sdf::Trajectory();
    newTraj->SetId(traj.id());
    newTraj->SetType(traj.type());
    newTraj->SetTension(traj.tension());
    for (int j = 0; j < traj.waypoints_size(); ++j)
    {
      const auto &point = traj.waypoints(j);
      auto newPoint = new sdf::Waypoint();
      newPoint->SetTime(point.time());
      newPoint->SetPose(msgs::Convert(point.pose()));
      newTraj->AddWaypoint(*newPoint);
    }
    out.AddTrajectory(*newTraj);
  }
  return out;
}

//////////////////////////////////////////////////
template<>
msgs::Light ignition::gazebo::convert(const sdf::Light &_in)
{
  msgs::Light out;
  out.set_name(_in.Name());
  msgs::Set(out.mutable_pose(), _in.RawPose());
  msgs::Set(out.mutable_diffuse(), _in.Diffuse());
  msgs::Set(out.mutable_specular(), _in.Specular());
  out.set_attenuation_constant(_in.ConstantAttenuationFactor());
  out.set_attenuation_linear(_in.LinearAttenuationFactor());
  out.set_attenuation_quadratic(_in.QuadraticAttenuationFactor());
  out.set_range(_in.AttenuationRange());
  msgs::Set(out.mutable_direction(), _in.Direction());
  out.set_cast_shadows(_in.CastShadows());
  out.set_spot_inner_angle(_in.SpotInnerAngle().Radian());
  out.set_spot_outer_angle(_in.SpotOuterAngle().Radian());
  out.set_spot_falloff(_in.SpotFalloff());
  if (_in.Type() == sdf::LightType::POINT)
    out.set_type(msgs::Light_LightType_POINT);
  else if (_in.Type() == sdf::LightType::SPOT)
    out.set_type(msgs::Light_LightType_SPOT);
  else if (_in.Type() == sdf::LightType::DIRECTIONAL)
    out.set_type(msgs::Light_LightType_DIRECTIONAL);
  return out;
}

//////////////////////////////////////////////////
template<>
sdf::Light ignition::gazebo::convert(const msgs::Light &_in)
{
  sdf::Light out;
  out.SetName(_in.name());
  out.SetRawPose(msgs::Convert(_in.pose()));
  out.SetDiffuse(msgs::Convert(_in.diffuse()));
  out.SetSpecular(msgs::Convert(_in.specular()));
  out.SetConstantAttenuationFactor(_in.attenuation_constant());
  out.SetLinearAttenuationFactor(_in.attenuation_linear());
  out.SetQuadraticAttenuationFactor(_in.attenuation_quadratic());
  out.SetAttenuationRange(_in.range());
  out.SetDirection(msgs::Convert(_in.direction()));
  out.SetCastShadows(_in.cast_shadows());
  out.SetSpotInnerAngle(math::Angle(_in.spot_inner_angle()));
  out.SetSpotOuterAngle(math::Angle(_in.spot_outer_angle()));
  out.SetSpotFalloff(_in.spot_falloff());
  if (_in.type() == msgs::Light_LightType_POINT)
    out.SetType(sdf::LightType::POINT);
  else if (_in.type() == msgs::Light_LightType_SPOT)
    out.SetType(sdf::LightType::SPOT);
  else if (_in.type() == msgs::Light_LightType_DIRECTIONAL)
    out.SetType(sdf::LightType::DIRECTIONAL);
  return out;
}

//////////////////////////////////////////////////
template<>
msgs::GUI ignition::gazebo::convert(const sdf::Gui &_in)
{
  msgs::GUI out;

  out.set_fullscreen(_in.Fullscreen());

  // Set gui plugins
  auto elem = _in.Element();
  if (elem && elem->HasElement("plugin"))
  {
    auto pluginElem = elem->GetElement("plugin");
    while (pluginElem)
    {
      auto pluginMsg = out.add_plugin();
      pluginMsg->set_name(pluginElem->Get<std::string>("name"));
      pluginMsg->set_filename(pluginElem->Get<std::string>("filename"));

      std::stringstream ss;
      for (auto innerElem = pluginElem->GetFirstElement();
          innerElem; innerElem = innerElem->GetNextElement(""))
      {
        ss << innerElem->ToString("");
      }
      pluginMsg->set_innerxml(ss.str());
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }

  if (elem->HasElement("camera"))
  {
    ignwarn << "<gui><camera> can't be converted yet" << std::endl;
  }

  return out;
}

//////////////////////////////////////////////////
template<>
msgs::Time ignition::gazebo::convert(
    const std::chrono::steady_clock::duration &_in)
{
  msgs::Time out;

  auto secNsec = ignition::math::durationToSecNsec(_in);

  out.set_sec(secNsec.first);
  out.set_nsec(secNsec.second);

  return out;
}

//////////////////////////////////////////////////
template<>
std::chrono::steady_clock::duration ignition::gazebo::convert(
    const msgs::Time &_in)
{
  return std::chrono::seconds(_in.sec()) + std::chrono::nanoseconds(_in.nsec());
}

//////////////////////////////////////////////////
template<>
msgs::Inertial ignition::gazebo::convert(const math::Inertiald &_in)
{
  msgs::Inertial out;
  msgs::Set(out.mutable_pose(), _in.Pose());
  out.set_mass(_in.MassMatrix().Mass());
  out.set_ixx(_in.MassMatrix().Ixx());
  out.set_iyy(_in.MassMatrix().Iyy());
  out.set_izz(_in.MassMatrix().Izz());
  out.set_ixy(_in.MassMatrix().Ixy());
  out.set_ixz(_in.MassMatrix().Ixz());
  out.set_iyz(_in.MassMatrix().Iyz());
  return out;
}

//////////////////////////////////////////////////
template<>
math::Inertiald ignition::gazebo::convert(const msgs::Inertial &_in)
{
  math::MassMatrix3d massMatrix;
  massMatrix.SetMass(_in.mass());
  massMatrix.SetIxx(_in.ixx());
  massMatrix.SetIyy(_in.iyy());
  massMatrix.SetIzz(_in.izz());
  massMatrix.SetIxy(_in.ixy());
  massMatrix.SetIxz(_in.ixz());
  massMatrix.SetIyz(_in.iyz());

  math::Inertiald out;
  out.SetMassMatrix(massMatrix);
  out.SetPose(msgs::Convert(_in.pose()));
  return out;
}

//////////////////////////////////////////////////
template<>
msgs::Axis ignition::gazebo::convert(const sdf::JointAxis &_in)
{
  msgs::Axis out;
  msgs::Set(out.mutable_xyz(), _in.Xyz());
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  out.set_use_parent_model_frame(_in.UseParentModelFrame());
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
  out.set_xyz_expressed_in(_in.XyzExpressedIn());
  out.set_damping(_in.Damping());
  out.set_friction(_in.Friction());
  out.set_limit_lower(_in.Lower());
  out.set_limit_upper(_in.Upper());
  out.set_limit_effort(_in.Effort());
  out.set_limit_velocity(_in.MaxVelocity());

  // TODO(anyone) Only on SDF:
  // * initial position
  // * spring reference
  // * spring stiffness
  // * stiffness
  // * dissipation
  // Only on msg:
  // * position
  // * velocity
  // * force
  // * acceleration

  return out;
}

//////////////////////////////////////////////////
template<>
sdf::JointAxis ignition::gazebo::convert(const msgs::Axis &_in)
{
  sdf::JointAxis out;
  out.SetXyz(msgs::Convert(_in.xyz()));
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  out.SetUseParentModelFrame(_in.use_parent_model_frame());
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
  out.SetXyzExpressedIn(_in.xyz_expressed_in());
  out.SetDamping(_in.damping());
  out.SetFriction(_in.friction());
  out.SetLower(_in.limit_lower());
  out.SetUpper(_in.limit_upper());
  out.SetEffort(_in.limit_effort());
  out.SetMaxVelocity(_in.limit_velocity());
  return out;
}

//////////////////////////////////////////////////
template<>
msgs::Scene ignition::gazebo::convert(const sdf::Scene &_in)
{
  msgs::Scene out;
  // todo(anyone) add Name to sdf::Scene?
  // out.set_name(_in.Name());
  msgs::Set(out.mutable_ambient(), _in.Ambient());
  msgs::Set(out.mutable_background(), _in.Background());
  out.set_shadows(_in.Shadows());
  out.set_grid(_in.Grid());
  out.set_origin_visual(_in.OriginVisual());
  return out;
}

//////////////////////////////////////////////////
template<>
sdf::Scene ignition::gazebo::convert(const msgs::Scene &_in)
{
  sdf::Scene out;
  // todo(anyone) add SetName to sdf::Scene?
  // out.SetName(_in.name());
  out.SetAmbient(msgs::Convert(_in.ambient()));
  out.SetBackground(msgs::Convert(_in.background()));
  out.SetShadows(_in.shadows());
  out.SetGrid(_in.grid());
  out.SetOriginVisual(_in.origin_visual());
  return out;
}

//////////////////////////////////////////////////
template<>
msgs::Atmosphere ignition::gazebo::convert(const sdf::Atmosphere &_in)
{
  msgs::Atmosphere out;
  out.set_temperature(_in.Temperature().Kelvin());
  out.set_pressure(_in.Pressure());
  if (_in.Type() == sdf::AtmosphereType::ADIABATIC)
  {
    out.set_type(msgs::Atmosphere::ADIABATIC);
  }
  // todo(anyone) add mass density to sdf::Atmosphere?
  // out.set_mass_density(_in.MassDensity());k

  return out;
}

//////////////////////////////////////////////////
template<>
sdf::Atmosphere ignition::gazebo::convert(const msgs::Atmosphere &_in)
{
  sdf::Atmosphere out;
  out.SetTemperature(math::Temperature(_in.temperature()));
  out.SetPressure(_in.pressure());
  // todo(anyone) add temperature gradient to msgs::Atmosphere?

  if (_in.type() == msgs::Atmosphere::ADIABATIC)
  {
    out.SetType(sdf::AtmosphereType::ADIABATIC);
  }

  return out;
}

//////////////////////////////////////////////////
void ignition::gazebo::set(msgs::Time *_msg,
    const std::chrono::steady_clock::duration &_in)
{
  auto secNsec = ignition::math::durationToSecNsec(_in);
  _msg->set_sec(secNsec.first);
  _msg->set_nsec(secNsec.second);
}

//////////////////////////////////////////////////
void ignition::gazebo::set(msgs::WorldStatistics *_msg,
    const gazebo::UpdateInfo &_in)
{
  set(_msg->mutable_sim_time(), _in.simTime);
  set(_msg->mutable_real_time(), _in.realTime);
  set(_msg->mutable_step_size(), _in.dt);
  _msg->set_iterations(_in.iterations);
  _msg->set_paused(_in.paused);
}

//////////////////////////////////////////////////
void ignition::gazebo::set(msgs::SensorNoise *_msg, const sdf::Noise &_sdf)
{
  switch (_sdf.Type())
  {
    case sdf::NoiseType::GAUSSIAN:
      _msg->set_type(msgs::SensorNoise::GAUSSIAN);
      break;
    case sdf::NoiseType::GAUSSIAN_QUANTIZED:
      _msg->set_type(msgs::SensorNoise::GAUSSIAN_QUANTIZED);
      break;

    case sdf::NoiseType::NONE:
    default:
      _msg->set_type(msgs::SensorNoise::NONE);
      break;
  }

  _msg->set_mean(_sdf.Mean());
  _msg->set_stddev(_sdf.StdDev());
  _msg->set_bias_mean(_sdf.BiasMean());
  _msg->set_bias_stddev(_sdf.BiasStdDev());
  _msg->set_precision(_sdf.Precision());
  _msg->set_dynamic_bias_stddev(_sdf.DynamicBiasStdDev());
  _msg->set_dynamic_bias_correlation_time(_sdf.DynamicBiasCorrelationTime());
}

//////////////////////////////////////////////////
template<>
sdf::Noise ignition::gazebo::convert(const msgs::SensorNoise &_in)
{
  sdf::Noise out;

  switch (_in.type())
  {
    case msgs::SensorNoise::GAUSSIAN:
      out.SetType(sdf::NoiseType::GAUSSIAN);
      break;
    case msgs::SensorNoise::GAUSSIAN_QUANTIZED:
      out.SetType(sdf::NoiseType::GAUSSIAN_QUANTIZED);
      break;

    case msgs::SensorNoise::NONE:
    default:
      out.SetType(sdf::NoiseType::NONE);
      break;
  }

  out.SetMean(_in.mean());
  out.SetStdDev(_in.stddev());
  out.SetBiasMean(_in.bias_mean());
  out.SetBiasStdDev(_in.bias_stddev());
  out.SetPrecision(_in.precision());
  out.SetDynamicBiasStdDev(_in.dynamic_bias_stddev());
  out.SetDynamicBiasCorrelationTime(_in.dynamic_bias_correlation_time());
  return out;
}


//////////////////////////////////////////////////
template<>
msgs::Sensor ignition::gazebo::convert(const sdf::Sensor &_in)
{
  msgs::Sensor out;
  out.set_name(_in.Name());
  out.set_type(_in.TypeStr());
  out.set_update_rate(_in.UpdateRate());
  out.set_topic(_in.Topic());
  msgs::Set(out.mutable_pose(), _in.RawPose());

  if (_in.Type() == sdf::SensorType::MAGNETOMETER)
  {
    if (_in.MagnetometerSensor())
    {
      msgs::MagnetometerSensor *sensor = out.mutable_magnetometer();
      if (_in.MagnetometerSensor()->XNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(sensor->mutable_x_noise(),
            _in.MagnetometerSensor()->XNoise());
      }
      if (_in.MagnetometerSensor()->YNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(sensor->mutable_y_noise(),
            _in.MagnetometerSensor()->YNoise());
      }
      if (_in.MagnetometerSensor()->ZNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(sensor->mutable_z_noise(),
            _in.MagnetometerSensor()->ZNoise());
      }
    }
    else
    {
      ignerr << "Attempting to convert a magnetometer SDF sensor, but the "
        << "sensor pointer is null.\n";
    }
  }
  else if (_in.Type() == sdf::SensorType::CAMERA ||
           _in.Type() == sdf::SensorType::DEPTH_CAMERA ||
           _in.Type() == sdf::SensorType::RGBD_CAMERA ||
           _in.Type() == sdf::SensorType::THERMAL_CAMERA)
  {
    if (_in.CameraSensor())
    {
      const sdf::Camera *sdfCam = _in.CameraSensor();
      msgs::CameraSensor *sensor = out.mutable_camera();
      sensor->set_horizontal_fov(sdfCam->HorizontalFov().Radian());
      sensor->mutable_image_size()->set_x(sdfCam->ImageWidth());
      sensor->mutable_image_size()->set_y(sdfCam->ImageHeight());
      sensor->set_near_clip(sdfCam->NearClip());
      sensor->set_far_clip(sdfCam->FarClip());
      sensor->set_save_enabled(sdfCam->SaveFrames());
      sensor->set_save_path(sdfCam->SaveFramesPath());
      sensor->set_image_format(sdfCam->PixelFormatStr());
      msgs::Distortion *dist = sensor->mutable_distortion();
      msgs::Set(dist->mutable_center(), sdfCam->DistortionCenter());
      dist->set_k1(sdfCam->DistortionK1());
      dist->set_k2(sdfCam->DistortionK2());
      dist->set_k3(sdfCam->DistortionK3());
      dist->set_p1(sdfCam->DistortionP1());
      dist->set_p2(sdfCam->DistortionP2());
    }
    else
    {
      ignerr << "Attempting to convert a camera SDF sensor, but the "
        << "sensor pointer is null.\n";
    }
  }
  else if (_in.Type() == sdf::SensorType::ALTIMETER)
  {
    if (_in.AltimeterSensor())
    {
      msgs::AltimeterSensor *sensor = out.mutable_altimeter();

      if (_in.AltimeterSensor()->VerticalPositionNoise().Type()
          != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(sensor->mutable_vertical_position_noise(),
            _in.AltimeterSensor()->VerticalPositionNoise());
      }
      if (_in.AltimeterSensor()->VerticalVelocityNoise().Type()
          != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(sensor->mutable_vertical_velocity_noise(),
            _in.AltimeterSensor()->VerticalVelocityNoise());
      }
    }
    else
    {
      ignerr << "Attempting to convert an altimeter SDF sensor, but the "
        << "sensor pointer is null.\n";
    }
  }
  else if (_in.Type() == sdf::SensorType::AIR_PRESSURE)
  {
    if (_in.AirPressureSensor())
    {
      msgs::AirPressureSensor *sensor = out.mutable_air_pressure();

      if (_in.AirPressureSensor()->PressureNoise().Type()
          != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(sensor->mutable_pressure_noise(),
            _in.AirPressureSensor()->PressureNoise());
      }
      sensor->set_reference_altitude(
          _in.AirPressureSensor()->ReferenceAltitude());
    }
    else
    {
      ignerr << "Attempting to convert an air pressure SDF sensor, but the "
        << "sensor pointer is null.\n";
    }
  }
  else if (_in.Type() == sdf::SensorType::IMU)
  {
    if (_in.ImuSensor())
    {
      const sdf::Imu *sdfImu = _in.ImuSensor();
      msgs::IMUSensor *sensor = out.mutable_imu();

      if (sdfImu->LinearAccelerationXNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(
            sensor->mutable_linear_acceleration()->mutable_x_noise(),
            sdfImu->LinearAccelerationXNoise());
      }
      if (sdfImu->LinearAccelerationYNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(
            sensor->mutable_linear_acceleration()->mutable_y_noise(),
            sdfImu->LinearAccelerationYNoise());
      }
      if (sdfImu->LinearAccelerationZNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(
            sensor->mutable_linear_acceleration()->mutable_z_noise(),
            sdfImu->LinearAccelerationZNoise());
      }

      if (sdfImu->AngularVelocityXNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(
            sensor->mutable_angular_velocity()->mutable_x_noise(),
            sdfImu->AngularVelocityXNoise());
      }
      if (sdfImu->AngularVelocityYNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(
            sensor->mutable_angular_velocity()->mutable_y_noise(),
            sdfImu->AngularVelocityYNoise());
      }
      if (sdfImu->AngularVelocityZNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(
            sensor->mutable_angular_velocity()->mutable_z_noise(),
            sdfImu->AngularVelocityZNoise());
      }
      sensor->mutable_orientation_ref_frame()->set_localization(
          sdfImu->Localization());

      msgs::Set(sensor->mutable_orientation_ref_frame()->mutable_custom_rpy(),
        sdfImu->CustomRpy());
      sensor->mutable_orientation_ref_frame()->set_custom_rpy_parent_frame(
          sdfImu->CustomRpyParentFrame());

      msgs::Set(
          sensor->mutable_orientation_ref_frame()->mutable_gravity_dir_x(),
          sdfImu->GravityDirX());
      sensor->mutable_orientation_ref_frame()->set_gravity_dir_x_parent_frame(
          sdfImu->GravityDirXParentFrame());
    }
    else
    {
      ignerr << "Attempting to convert an IMU SDF sensor, but the "
        << "sensor pointer is null.\n";
    }
  }
  else if (_in.Type() == sdf::SensorType::LIDAR ||
           _in.Type() == sdf::SensorType::GPU_LIDAR)
  {
    if (_in.LidarSensor())
    {
      const sdf::Lidar *sdfLidar = _in.LidarSensor();
      msgs::LidarSensor *sensor = out.mutable_lidar();

      if (sdfLidar->LidarNoise().Type() != sdf::NoiseType::NONE)
      {
        ignition::gazebo::set(sensor->mutable_noise(), sdfLidar->LidarNoise());
      }
      sensor->set_horizontal_samples(sdfLidar->HorizontalScanSamples());
      sensor->set_horizontal_resolution(sdfLidar->HorizontalScanResolution());
      sensor->set_horizontal_min_angle(
          sdfLidar->HorizontalScanMinAngle().Radian());
      sensor->set_horizontal_max_angle(
          sdfLidar->HorizontalScanMaxAngle().Radian());

      sensor->set_vertical_samples(sdfLidar->VerticalScanSamples());
      sensor->set_vertical_resolution(sdfLidar->VerticalScanResolution());
      sensor->set_vertical_min_angle(sdfLidar->VerticalScanMinAngle().Radian());
      sensor->set_vertical_max_angle(sdfLidar->VerticalScanMaxAngle().Radian());

      sensor->set_range_min(sdfLidar->RangeMin());
      sensor->set_range_max(sdfLidar->RangeMax());
      sensor->set_range_resolution(sdfLidar->RangeResolution());
    }
    else
    {
      ignerr << "Attempting to convert a Lidar SDF sensor, but the "
        << "sensor pointer is null.\n";
    }
  }
  return out;
}

//////////////////////////////////////////////////
template<>
sdf::Sensor ignition::gazebo::convert(const msgs::Sensor &_in)
{
  sdf::Sensor out;
  out.SetName(_in.name());
  if (!out.SetType(_in.type()))
    ignerr << "Failed to set the sensor type from [" << _in.type() << "]\n";

  out.SetUpdateRate(_in.update_rate());
  out.SetTopic(_in.topic());
  out.SetRawPose(msgs::Convert(_in.pose()));
  if (out.Type() == sdf::SensorType::MAGNETOMETER)
  {
    sdf::Magnetometer sensor;
    if (_in.has_magnetometer())
    {
      if (_in.magnetometer().has_x_noise())
      {
        sensor.SetXNoise(ignition::gazebo::convert<sdf::Noise>(
              _in.magnetometer().x_noise()));
      }
      if (_in.magnetometer().has_y_noise())
      {
        sensor.SetYNoise(ignition::gazebo::convert<sdf::Noise>(
              _in.magnetometer().y_noise()));
      }
      if (_in.magnetometer().has_z_noise())
      {
        sensor.SetZNoise(ignition::gazebo::convert<sdf::Noise>(
              _in.magnetometer().z_noise()));
      }
    }
    else
    {
      ignerr << "Attempting to convert a magnetometer sensor message, but the "
        << "message does not have a magnetometer nested message.\n";
    }

    out.SetMagnetometerSensor(sensor);
  }
  else if (out.Type() == sdf::SensorType::CAMERA ||
           out.Type() == sdf::SensorType::DEPTH_CAMERA ||
           out.Type() == sdf::SensorType::RGBD_CAMERA ||
           out.Type() == sdf::SensorType::THERMAL_CAMERA)
  {
    sdf::Camera sensor;

    if (_in.has_camera())
    {
      sensor.SetHorizontalFov(_in.camera().horizontal_fov());
      sensor.SetImageWidth(_in.camera().image_size().x());
      sensor.SetImageHeight(_in.camera().image_size().y());
      sensor.SetPixelFormatStr(_in.camera().image_format());
      sensor.SetNearClip(_in.camera().near_clip());
      sensor.SetFarClip(_in.camera().far_clip());
      sensor.SetSaveFrames(_in.camera().save_enabled());
      sensor.SetSaveFramesPath(_in.camera().save_path());
      if (_in.camera().has_distortion())
      {
        sensor.SetDistortionK1(_in.camera().distortion().k1());
        sensor.SetDistortionK2(_in.camera().distortion().k2());
        sensor.SetDistortionK3(_in.camera().distortion().k3());
        sensor.SetDistortionP1(_in.camera().distortion().p1());
        sensor.SetDistortionP2(_in.camera().distortion().p2());
        sensor.SetDistortionCenter(
            msgs::Convert(_in.camera().distortion().center()));
      }
    }
    else
    {
      ignerr << "Attempting to convert a camera sensor message, but the "
        << "message does not have a camera nested message.\n";
    }

    out.SetCameraSensor(sensor);
  }
  else if (out.Type() == sdf::SensorType::ALTIMETER)
  {
    sdf::Altimeter sensor;
    if (_in.has_altimeter())
    {
      if (_in.altimeter().has_vertical_position_noise())
      {
        sensor.SetVerticalPositionNoise(ignition::gazebo::convert<sdf::Noise>(
              _in.altimeter().vertical_position_noise()));
      }

      if (_in.altimeter().has_vertical_velocity_noise())
      {
        sensor.SetVerticalVelocityNoise(ignition::gazebo::convert<sdf::Noise>(
              _in.altimeter().vertical_velocity_noise()));
      }
    }
    else
    {
      ignerr << "Attempting to convert an altimeter sensor message, but the "
        << "message does not have a altimeter nested message.\n";
    }

    out.SetAltimeterSensor(sensor);
  }
  else if (out.Type() == sdf::SensorType::AIR_PRESSURE)
  {
    sdf::AirPressure sensor;
    if (_in.has_air_pressure())
    {
      if (_in.air_pressure().has_pressure_noise())
      {
        sensor.SetPressureNoise(ignition::gazebo::convert<sdf::Noise>(
              _in.air_pressure().pressure_noise()));
      }

      sensor.SetReferenceAltitude(_in.air_pressure().reference_altitude());
    }
    else
    {
      ignerr << "Attempting to convert an air pressure sensor message, but the "
        << "message does not have an air pressure nested message.\n";
    }

    out.SetAirPressureSensor(sensor);
  }
  else if (out.Type() == sdf::SensorType::IMU)
  {
    sdf::Imu sensor;
    if (_in.has_imu())
    {
      if (_in.imu().has_linear_acceleration())
      {
        if (_in.imu().linear_acceleration().has_x_noise())
        {
          sensor.SetLinearAccelerationXNoise(
              ignition::gazebo::convert<sdf::Noise>(
                _in.imu().linear_acceleration().x_noise()));
        }
        if (_in.imu().linear_acceleration().has_y_noise())
        {
          sensor.SetLinearAccelerationYNoise(
              ignition::gazebo::convert<sdf::Noise>(
                _in.imu().linear_acceleration().y_noise()));
        }
        if (_in.imu().linear_acceleration().has_z_noise())
        {
          sensor.SetLinearAccelerationZNoise(
              ignition::gazebo::convert<sdf::Noise>(
                _in.imu().linear_acceleration().z_noise()));
        }
      }

      if (_in.imu().has_angular_velocity())
      {
        if (_in.imu().angular_velocity().has_x_noise())
        {
          sensor.SetAngularVelocityXNoise(
              ignition::gazebo::convert<sdf::Noise>(
                _in.imu().angular_velocity().x_noise()));
        }
        if (_in.imu().angular_velocity().has_y_noise())
        {
          sensor.SetAngularVelocityYNoise(
              ignition::gazebo::convert<sdf::Noise>(
                _in.imu().angular_velocity().y_noise()));
        }
        if (_in.imu().angular_velocity().has_z_noise())
        {
          sensor.SetAngularVelocityZNoise(
              ignition::gazebo::convert<sdf::Noise>(
                _in.imu().angular_velocity().z_noise()));
        }
      }

      if (_in.imu().has_orientation_ref_frame())
      {
        sensor.SetLocalization(
            _in.imu().orientation_ref_frame().localization());

        if (_in.imu().orientation_ref_frame().has_custom_rpy())
        {
          sensor.SetCustomRpy(
              msgs::Convert(_in.imu().orientation_ref_frame().custom_rpy()));
          sensor.SetCustomRpyParentFrame(
              _in.imu().orientation_ref_frame().custom_rpy_parent_frame());
        }

        if (_in.imu().orientation_ref_frame().has_gravity_dir_x())
        {
          sensor.SetGravityDirX(msgs::Convert(
                _in.imu().orientation_ref_frame().gravity_dir_x()));
          sensor.SetGravityDirXParentFrame(
              _in.imu().orientation_ref_frame().gravity_dir_x_parent_frame());
        }
      }
    }
    else
    {
      ignerr << "Attempting to convert an IMU sensor message, but the "
        << "message does not have an IMU nested message.\n";
    }

    out.SetImuSensor(sensor);
  }
  else if (out.Type() == sdf::SensorType::GPU_LIDAR ||
           out.Type() == sdf::SensorType::LIDAR)
  {
    sdf::Lidar sensor;
    if (_in.has_lidar())
    {
      sensor.SetHorizontalScanSamples(_in.lidar().horizontal_samples());
      sensor.SetHorizontalScanResolution(_in.lidar().horizontal_resolution());
      sensor.SetHorizontalScanMinAngle(_in.lidar().horizontal_min_angle());
      sensor.SetHorizontalScanMaxAngle(_in.lidar().horizontal_max_angle());

      sensor.SetVerticalScanSamples(_in.lidar().vertical_samples());
      sensor.SetVerticalScanResolution(_in.lidar().vertical_resolution());
      sensor.SetVerticalScanMinAngle(_in.lidar().vertical_min_angle());
      sensor.SetVerticalScanMaxAngle(_in.lidar().vertical_max_angle());

      sensor.SetRangeMin(_in.lidar().range_min());
      sensor.SetRangeMax(_in.lidar().range_max());
      sensor.SetRangeResolution(_in.lidar().range_resolution());

      if (_in.lidar().has_noise())
      {
        sensor.SetLidarNoise(ignition::gazebo::convert<sdf::Noise>(
              _in.lidar().noise()));
      }
    }
    else
    {
      ignerr << "Attempting to convert a lidar sensor message, but the "
        << "message does not have a lidar nested message.\n";
    }

    out.SetLidarSensor(sensor);
  }
  return out;
}

//////////////////////////////////////////////////
template<>
msgs::WorldStatistics ignition::gazebo::convert(const gazebo::UpdateInfo &_in)
{
  msgs::WorldStatistics out;
  set(&out, _in);
  return out;
}

//////////////////////////////////////////////////
template<>
gazebo::UpdateInfo ignition::gazebo::convert(const msgs::WorldStatistics &_in)
{
  gazebo::UpdateInfo out;
  out.iterations = _in.iterations();
  out.paused = _in.paused();
  out.simTime = convert<std::chrono::steady_clock::duration>(_in.sim_time());
  out.realTime = convert<std::chrono::steady_clock::duration>(_in.real_time());
  out.dt = convert<std::chrono::steady_clock::duration>(_in.step_size());
  return out;
}
