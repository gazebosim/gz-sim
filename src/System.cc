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
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>
#include "ignition/gazebo/System.hh"

using namespace ignition::gazebo;

// Private data class
class ignition::gazebo::SystemPrivate
{
  public: bool SceneService(ignition::msgs::Scene &_rep);

  /// \brief Communication node.
  public: ignition::transport::Node node;
};

/////////////////////////////////////////////////
System::System()
: dataPtr(new SystemPrivate())
{
  this->dataPtr->node.Advertise("/ign/gazebo/scene",
                                &SystemPrivate::SceneService, this->dataPtr);
}

/////////////////////////////////////////////////
System::~System()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
void System::EntityCreated(const Entity &/*_entity*/)
{
}

/////////////////////////////////////////////////
bool System::Update()
{
  return false;
}

//////////////////////////////////////////////////
// SystemPrivate functions
//////////////////////////////////////////////////

//////////////////////////////////////////////////
bool SystemPrivate::SceneService(ignition::msgs::Scene &_rep)
{
  _rep.set_name("gazebo");
  ignition::msgs::Model *model = _rep.add_model();

  model->set_name("sphere");
  model->set_id(0);
  ignition::msgs::Set(model->mutable_pose(),
                      ignition::math::Pose3d(0, 0, 1, 0, 0, 0));

  ignition::msgs::Link *link = model->add_link();
  link->set_name("link");

  ignition::msgs::Visual *visual = link->add_visual();
  visual->set_name("visual");

  ignition::msgs::Geometry *geom = visual->mutable_geometry();
  geom->set_type(ignition::msgs::Geometry::SPHERE);
  ignition::msgs::SphereGeom *sphere = geom->mutable_sphere();
  sphere->set_radius(1.0);

  return true;
}
