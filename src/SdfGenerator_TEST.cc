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

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <ignition/common/Console.hh>
#include <ignition/fuel_tools/ClientConfig.hh>
#include <ignition/fuel_tools/Interface.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/EventManager.hh"
#include "ignition/gazebo/SdfEntityCreator.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/test_config.hh"

#include "helpers/UniqueTestDirectoryEnv.hh"

#include "SdfGenerator.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
/// \breif Checks if elemA is a subset of elemB
static bool isSubset(const sdf::ElementPtr &_elemA,
                     const sdf::ElementPtr &_elemB)
{
  if (_elemA->GetName() != _elemB->GetName())
    return false;
  if (_elemA->GetAttributeCount() != _elemB->GetAttributeCount())
    return false;

  // Compare attributes
  for (std::size_t i = 0; i < _elemA->GetAttributeCount(); ++i)
  {
    sdf::ParamPtr attrA = _elemA->GetAttribute(i);
    sdf::ParamPtr attrB = _elemB->GetAttribute(attrA->GetKey());
    if (attrA->GetTypeName() != attrB->GetTypeName())
      return false;
    if (attrA->GetAsString() != attrB->GetAsString())
      return false;
  }
  // Compare values
  {
    sdf::ParamPtr valA = _elemA->GetValue();
    if (nullptr != valA)
    {
      sdf::ParamPtr valB = _elemB->GetValue();
      if (nullptr == valB)
        return false;
      if (valA->GetTypeName() != valB->GetTypeName())
        return false;
      if (valA->GetAsString() != valB->GetAsString())
        return false;
    }
  }

  auto elemBClone = _elemB->Clone();
  for (sdf::ElementPtr childElemA = _elemA->GetFirstElement(); childElemA;
       childElemA = childElemA->GetNextElement())
  {
    bool result = false;
    // Since we want to ignore order of elements, we have to compare against
    // every element in _elemB.
    for (sdf::ElementPtr childElemB = elemBClone->GetFirstElement(); childElemB;
         childElemB = childElemB->GetNextElement())
    {
      result = result || isSubset(childElemA, childElemB);
      if (result)
      {
        // Remove the matching element so that we don't compare against it
        // again.
        elemBClone->RemoveChild(childElemB);
        break;
      }
    }

    if (!result)
      return false;
  }

  return true;
}

/////////////////////////////////////////////////
TEST(CompareElements, CompareWithDuplicateElements)
{
  const std::string m1Sdf = R"(
  <sdf version="1.7">
    <model name="M1">
      <pose>0 0 0 0 0 0 0</pose>
    </model>
  </sdf>
  )";
  const std::string m1CompTestSdf = R"(
  <sdf version="1.7">
    <model name="M1">
      <pose>0 0 0 0 0 0 0</pose>
      <pose>0 0 0 0 0 0 0</pose>
    </model>
  </sdf>
  )";
  auto m1 = std::make_shared<sdf::Element>();
  auto m1CompTest = std::make_shared<sdf::Element>();
  sdf::initFile("model.sdf", m1);
  sdf::initFile("model.sdf", m1CompTest);
  sdf::readString(m1Sdf, m1);
  sdf::readString(m1CompTestSdf, m1CompTest);

  EXPECT_TRUE(isSubset(m1, m1CompTest));
  EXPECT_FALSE(isSubset(m1CompTest, m1));
}

/////////////////////////////////////////////////
class ElementUpdateFixture : public ::testing::Test
{
  public: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);

    fuel_tools::ClientConfig config;
    config.SetCacheLocation(test::UniqueTestDirectoryEnv::Path());
    this->fuelClient = std::make_unique<fuel_tools::FuelClient>(config);

    auto fuelCb = [&](const std::string &_uri)
    {
      auto out =
          fuel_tools::fetchResourceWithClient(_uri, *this->fuelClient.get());
      if (!out.empty())
      {
        this->includeUriMap[out] = _uri;
      }
      return out;
    };

    // Configure SDF to fetch assets from ignition fuel.
    sdf::setFindCallback(fuelCb);
    creator = std::make_unique<SdfEntityCreator>(this->ecm, this->evm);
  }

  public: virtual void LoadWorld(const std::string &_path)
  {
    auto errors = this->root.Load(common::joinPaths(PROJECT_SOURCE_PATH,
        _path));

    ASSERT_EQ(1u, root.WorldCount());
    this->world = root.WorldByIndex(0);
    ASSERT_NE(nullptr, this->world);
    this->creator->CreateEntities(world);
  }

  public: virtual void LoadWorldString(const std::string &_worldSdf)
  {
    auto errors = this->root.LoadSdfString(_worldSdf);

    ASSERT_EQ(1u, root.WorldCount());
    this->world = root.WorldByIndex(0);
    ASSERT_NE(nullptr, this->world);
    this->creator->CreateEntities(world);
  }
  // Helper function to get a model by name from an sdf::World
  const sdf::Model *ModelByName(const std::string &_name)
  {
    for (std::size_t i = 0; i < this->world->ModelCount(); ++i)
    {
      if (_name == this->world->ModelByIndex(i)->Name())
      {
        return this->world->ModelByIndex(i);
      }
    }
    return nullptr;
  }

  public: EntityComponentManager ecm;
  public: EventManager evm;
  public: sdf::Root root;
  public: const sdf::World *world{nullptr};

  public: std::unique_ptr<SdfEntityCreator> creator;
  public: msgs::SdfGeneratorConfig sdfGenConfig;
  public: sdf_generator::IncludeUriMap includeUriMap;
  public: std::unique_ptr<fuel_tools::FuelClient> fuelClient;
};

/////////////////////////////////////////////////
class ModelElementFixture : public ElementUpdateFixture
{
  public: sdf::ElementPtr CreateModelElem(const std::string &_modelName)
  {
    using namespace sdf_generator;

    Entity model = this->ecm.EntityByComponents(
        components::Model(), components::Name(_modelName));
    if (kNullEntity == model)
      return nullptr;

    auto elem = std::make_shared<sdf::Element>();
    sdf::initFile("model.sdf", elem);
    updateModelElement(elem, this->ecm, model);
    return elem;
  }

  public: void TestModel(const std::string &_modelName)
  {
    auto elem = this->CreateModelElem(_modelName);
    ASSERT_NE(nullptr, elem);
    const sdf::Model *sdfModel = this->ModelByName(_modelName);
    ASSERT_NE(nullptr, sdfModel);
    EXPECT_TRUE(isSubset(sdfModel->Element(), elem));
    EXPECT_TRUE(isSubset(elem, sdfModel->Element()));
  }
};

/////////////////////////////////////////////////
TEST_F(ModelElementFixture, ModelsInline)
{
  this->LoadWorld("test/worlds/shapes.sdf");
  this->TestModel("box");
  this->TestModel("cylinder");
  this->TestModel("sphere");
}

/////////////////////////////////////////////////
TEST_F(ModelElementFixture, ModelIncluded)
{
  this->LoadWorld("test/worlds/save_world.sdf");
  this->TestModel("backpack1");
  this->TestModel("backpack2");
}

/////////////////////////////////////////////////
TEST_F(ModelElementFixture, ModelComponentUpdate)
{
  this->LoadWorld("test/worlds/shapes.sdf");
  std::string modelName = "box";
  Entity modelEntity = this->ecm.EntityByComponents(
      components::Model(), components::Name(modelName));

  auto *poseComp = this->ecm.Component<components::Pose>(modelEntity);

  math::Pose3d newPose{0.1, 0.2, 0.3, 0, 0, 0};
  EXPECT_NE(poseComp->Data(), newPose);

  *poseComp = components::Pose(newPose);

  auto modelElem = this->CreateModelElem(modelName);
  ASSERT_NE(nullptr, modelElem);
  // Check that the generated element has the new pose
  EXPECT_EQ(newPose, modelElem->Get<math::Pose3d>("pose"));
}

/////////////////////////////////////////////////
/// Test to show that CopyFrom and MergeFrom don't work.
TEST_F(ElementUpdateFixture, ConfigOverrideCopyOrMerge)
{
  {
    msgs::SdfGeneratorConfig::EntityGeneratorConfig globalConfig, modelConfig;
    globalConfig.mutable_expand_include_tags()->set_data(true);
    globalConfig.mutable_save_fuel_version()->set_data(true);

    modelConfig.mutable_expand_include_tags()->set_data(false);

    {
      auto combinedConfig = globalConfig;
      std::cout << "Before Merge global:\n"
                << globalConfig.DebugString() << std::endl;
      std::cout << "Before Merge model:\n"
                << modelConfig.DebugString() << std::endl;
      combinedConfig.MergeFrom(modelConfig);
      std::cout << "After Merge:\n"
                << combinedConfig.DebugString() << std::endl;
    }
    {
      auto combinedConfig = modelConfig;
      std::cout << "Before Merge2 global:\n"
                << globalConfig.DebugString() << std::endl;
      std::cout << "Before Merge2 model:\n"
                << modelConfig.DebugString() << std::endl;
      combinedConfig.MergeFrom(globalConfig);
      std::cout << "After Merge:\n"
                << combinedConfig.DebugString() << std::endl;
    }

    {
      auto combinedConfig = globalConfig;
      std::cout << "Before Copy global:\n"
                << globalConfig.DebugString() << std::endl;
      std::cout << "Before Copy model:\n"
                << modelConfig.DebugString() << std::endl;
      combinedConfig.CopyFrom(modelConfig);
      std::cout << "After Copy:\n" << combinedConfig.DebugString() << std::endl;
    }
  }

  {
    msgs::SdfGeneratorConfig::EntityGeneratorConfig globalConfig, modelConfig;
    globalConfig.mutable_expand_include_tags()->set_data(false);
    globalConfig.mutable_save_fuel_version()->set_data(false);

    modelConfig.mutable_save_fuel_version()->set_data(true);

    {
      auto combinedConfig = globalConfig;
      std::cout << "Before Merge global:\n"
                << globalConfig.DebugString() << std::endl;
      std::cout << "Before Merge model:\n"
                << modelConfig.DebugString() << std::endl;
      combinedConfig.MergeFrom(modelConfig);
      std::cout << "After Merge:\n"
                << combinedConfig.DebugString() << std::endl;
    }

    {
      auto combinedConfig = globalConfig;
      std::cout << "Before Copy global:\n"
                << globalConfig.DebugString() << std::endl;
      std::cout << "Before Copy model:\n"
                << modelConfig.DebugString() << std::endl;
      combinedConfig.CopyFrom(modelConfig);
      std::cout << "After Copy:\n" << combinedConfig.DebugString() << std::endl;
    }
  }
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, ConfigOverride)
{
  const std::string worldFile{"test/worlds/save_world.sdf"};
  this->LoadWorld(worldFile);
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  {
    this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_expand_include_tags()
      ->set_data(true);

    this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_save_fuel_version()
      ->set_data(true);

    auto &modelGenConfig =
        *this->sdfGenConfig.mutable_override_entity_gen_configs();
    modelGenConfig["save_world::backpack2"]
        .mutable_expand_include_tags()
        ->set_data(false);

    auto elem = std::make_shared<sdf::Element>();
    sdf::initFile("world.sdf", elem);
    sdf_generator::updateWorldElement(
        elem, this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);
    ASSERT_TRUE(elem->HasElement("include"));
    auto inclElem = elem->GetElement("include");
    EXPECT_EQ("backpack2", inclElem->Get<std::string>("name"));
  }

  {
    this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_expand_include_tags()
      ->set_data(false);

    this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_save_fuel_version()
      ->set_data(false);

    auto &modelGenConfig =
        *this->sdfGenConfig.mutable_override_entity_gen_configs();
    modelGenConfig["save_world::backpack2"]
        .mutable_save_fuel_version()
        ->set_data(true);

    auto elem = std::make_shared<sdf::Element>();
    sdf::initFile("world.sdf", elem);
    sdf_generator::updateWorldElement(
        elem, this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);
    ASSERT_TRUE(elem->HasElement("include"));
    auto inclElem = elem->GetElement("include");
    EXPECT_EQ("backpack1", inclElem->Get<std::string>("name"));
    inclElem = inclElem->GetNextElement("include");
    EXPECT_EQ("backpack2", inclElem->Get<std::string>("name"));
    auto uri = inclElem->Get<std::string>("uri");
    EXPECT_FALSE(uri.empty());
    const std::string version = common::split(uri, "/").back();
    EXPECT_NO_THROW(std::stol(version));
  }
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, WorldWithModelsInline)
{
  const std::string worldFile{"test/worlds/shapes.sdf"};
  this->LoadWorld(worldFile);
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  auto elem = std::make_shared<sdf::Element>();
  sdf::initFile("world.sdf", elem);
  sdf_generator::updateWorldElement(elem, this->ecm, worldEntity);

  EXPECT_TRUE(isSubset(this->root.WorldByIndex(0)->Element(), elem));
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, WorldWithModelsIncludedExpanded)
{
  this->LoadWorld("test/worlds/save_world.sdf");
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  auto elem = std::make_shared<sdf::Element>();
  sdf::initFile("world.sdf", elem);
  this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_expand_include_tags()
      ->set_data(true);
  sdf_generator::updateWorldElement(
      elem, this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);
  auto origWorldElem = this->root.WorldByIndex(0)->Element();
  EXPECT_TRUE(isSubset(origWorldElem, elem))
      << "Original: " << origWorldElem->ToString("")
      << "\n Generated: " << elem->ToString("");
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, WorldComponentUpdate)
{
  const std::string worldFile{"test/worlds/shapes.sdf"};
  this->LoadWorld(worldFile);
  std::string modelName = "box";
  Entity modelEntity = this->ecm.EntityByComponents(
      components::Model(), components::Name(modelName));

  auto *poseComp = this->ecm.Component<components::Pose>(modelEntity);

  math::Pose3d newPose{0.1, 0.2, 0.3, 0, 0, 0};
  EXPECT_NE(poseComp->Data(), newPose);

  *poseComp = components::Pose(newPose);

  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  auto elem = std::make_shared<sdf::Element>();
  sdf::initFile("world.sdf", elem);
  sdf_generator::updateWorldElement(elem, this->ecm, worldEntity);

  ASSERT_TRUE(elem->HasElement("model"));
  auto modelElem = elem->GetElement("model");
  for (; modelElem; modelElem->GetNextElement("model"))
  {
    if (modelName == modelElem->Get<std::string>("name"))
      break;
  }

  // Check that the generated element has the new pose
  EXPECT_EQ(newPose, modelElem->Get<math::Pose3d>("pose"));
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, WorldWithModelsIncludedNotExpanded)
{
  const std::string worldFile{"test/worlds/save_world.sdf"};
  this->LoadWorld(worldFile);
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  auto elem = std::make_shared<sdf::Element>();
  sdf::initFile("world.sdf", elem);
  this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_expand_include_tags()
      ->set_data(false);
  auto &modelGenConfig =
      *this->sdfGenConfig.mutable_override_entity_gen_configs();
  modelGenConfig["save_world::backpack3"]
      .mutable_save_fuel_version()
      ->set_data(true);
  sdf_generator::updateWorldElement(
      elem, this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);

  tinyxml2::XMLDocument originalSdfDoc;
  EXPECT_EQ(tinyxml2::XML_SUCCESS,
            originalSdfDoc.LoadFile(
                common::joinPaths(PROJECT_SOURCE_PATH, worldFile).c_str()));

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(elem->ToString("").c_str());

  // Compare elements from the original sdf xml and the generated xml
  auto origWorld = originalSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, origWorld);
  auto genWorld = genSdfDoc.RootElement();
  ASSERT_NE(nullptr, genWorld);

  // Check name
  {
    ASSERT_NE(nullptr, origWorld->Attribute("name"));
    ASSERT_NE(nullptr, genWorld->Attribute("name"));
    EXPECT_STREQ(origWorld->Attribute("name"), genWorld->Attribute("name"));
  }
  // Check include tag
  auto origInclude = origWorld->FirstChildElement("include");
  ASSERT_NE(nullptr, origInclude);
  auto genInclude = genWorld->FirstChildElement("include");
  for (; nullptr != origInclude;
       origInclude = origInclude->NextSiblingElement("include"),
       genInclude = genInclude->NextSiblingElement("include"))
  {
    ASSERT_NE(nullptr, genInclude);

    auto origUri = origInclude->FirstChildElement("uri");
    ASSERT_NE(nullptr, origUri);
    auto genUri = genInclude->FirstChildElement("uri");
    ASSERT_NE(nullptr, genUri);
    EXPECT_STREQ(origUri->GetText(), genUri->GetText());
  }
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, WorldWithModelsIncludedWithInvalidUris)
{
  const std::string goodUri =
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Backpack/1";

  // These are URIs that are potentially problematic.
  const std::vector<std::string> fuelUris = {
      // Thes following two URIs are valid, but have a trailing '/'
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Backpack/",
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Backpack/1/",
      // Thes following two URIs are invalid, and will not be saved
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Backpack/"
      "notInt",
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Backpack/"
      "notInt/",
  };

  std::string worldSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="invalid_uris">
    <include>
      <uri>)" + fuelUris[0] + R"(</uri>
      <name>backpack1</name>
    </include>
    <include>
      <uri>)" + fuelUris[1] + R"(</uri>
      <name>backpack2</name>
    </include>
    <include>
      <uri>)" + fuelUris[2] + R"(</uri>
      <name>backpack3</name>
    </include>
    <include>
      <uri>)" + fuelUris[3] + R"(</uri>
      <name>backpack3</name>
    </include>
  </world>
</sdf>
  )";

  this->LoadWorldString(worldSdf);
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  auto elem = std::make_shared<sdf::Element>();
  sdf::initFile("world.sdf", elem);
  auto globalEntityConfig =
      this->sdfGenConfig.mutable_global_entity_gen_config();
  globalEntityConfig->mutable_expand_include_tags()->set_data(false);
  globalEntityConfig->mutable_save_fuel_version()->set_data(true);
  sdf_generator::updateWorldElement(
      elem, this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);

  std::size_t count = 0;
  EXPECT_TRUE(elem->HasElement("include"));
  for (sdf::ElementPtr incl = elem->GetElement("include"); incl;
       incl = incl->GetNextElement("include"), ++count)
  {
    EXPECT_TRUE(incl->HasElement("uri"));
    EXPECT_EQ(goodUri, incl->Get<std::string>("uri"));
  }

  EXPECT_EQ(2u, count);
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, WorldWithModelsIncludedWithNonFuelUris)
{
  const std::vector<std::string> includeUris = {
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Backpack",
      std::string("file://") + PROJECT_SOURCE_PATH +
          "/test/worlds/models/sphere"};

  std::string worldSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="nonfuel_uris">
    <include>
      <uri>)" + includeUris[0] + R"(</uri>
      <name>model1</name>
    </include>
    <include>
      <uri>)" + includeUris[1] + R"(</uri>
      <name>model2</name>
    </include>
    <model name="model3">
      <link name="link3"/>
    </model>
  </world>
</sdf>
  )";

  this->LoadWorldString(worldSdf);
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  // There should be three models
  EXPECT_EQ(3u, this->ecm.EntitiesByComponents(components::Model()).size());

  auto elem = std::make_shared<sdf::Element>();
  sdf::initFile("world.sdf", elem);
  this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_expand_include_tags()
      ->set_data(false);
  // Model2 is not a Fuel model, so this should have no effect.
  auto &modelGenConfig =
      *this->sdfGenConfig.mutable_override_entity_gen_configs();
  modelGenConfig["save_world::model2"]
      .mutable_save_fuel_version()
      ->set_data(true);
  sdf_generator::updateWorldElement(
      elem, this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);

  std::cout << elem->ToString("") << std::endl;

  std::size_t count = 0;
  EXPECT_TRUE(elem->HasElement("include"));
  for (sdf::ElementPtr incl = elem->GetElement("include"); incl;
       incl = incl->GetNextElement("include"), ++count)
  {
    EXPECT_TRUE(incl->HasElement("uri"));
    EXPECT_EQ(includeUris[count], incl->Get<std::string>("uri"));
  }

  EXPECT_EQ(2u, count);
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, WorldWithModelsIncludedWithOneExpanded)
{
  const std::string worldFile{"test/worlds/save_world.sdf"};
  this->LoadWorld(worldFile);
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  auto elem = std::make_shared<sdf::Element>();
  sdf::initFile("world.sdf", elem);

  this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_expand_include_tags()
      ->set_data(false);
  auto &modelGenConfig =
      *this->sdfGenConfig.mutable_override_entity_gen_configs();
  modelGenConfig["save_world::backpack2"]
      .mutable_expand_include_tags()
      ->set_data(true);
  sdf_generator::updateWorldElement(
      elem, this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);

  tinyxml2::XMLDocument originalSdfDoc;
  EXPECT_EQ(tinyxml2::XML_SUCCESS,
            originalSdfDoc.LoadFile(
                common::joinPaths(PROJECT_SOURCE_PATH, worldFile).c_str()));

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(elem->ToString("").c_str());

  // Compare elements from the original sdf xml and the generated xml
  auto origWorld = originalSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, origWorld);
  auto genWorld = genSdfDoc.RootElement();
  ASSERT_NE(nullptr, genWorld);

  // Check include tag
  auto origInclude = origWorld->FirstChildElement("include");
  ASSERT_NE(nullptr, origInclude);
  auto genInclude = genWorld->FirstChildElement("include");
  ASSERT_NE(nullptr, genInclude);
  {
    auto origUri = origInclude->FirstChildElement("uri");
    ASSERT_NE(nullptr, origUri);
    auto genUri = genInclude->FirstChildElement("uri");
    ASSERT_NE(nullptr, genUri);
    EXPECT_STREQ(origUri->GetText(), genUri->GetText());
  }

  tinyxml2::XMLElement *backpack2Model{nullptr};
  for (auto modelElem = genWorld->FirstChildElement("model"); modelElem;
       modelElem = modelElem->NextSiblingElement("model"))
  {
    if (std::string("backpack2") == modelElem->Attribute("name"))
    {
      backpack2Model = modelElem;
      break;
    }
  }
  // Check that backpack2 is expanded
  ASSERT_NE(nullptr, backpack2Model);
  ASSERT_NE(nullptr, backpack2Model->Attribute("name"));
  EXPECT_STREQ(backpack2Model->Attribute("name"), "backpack2");
  ASSERT_NE(nullptr, backpack2Model->FirstChildElement("pose"));
  EXPECT_STREQ(backpack2Model->FirstChildElement("pose")->GetText(),
               "1 2 3 0.1 0.2 0.3");
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, WorldWithModelsExpandedWithOneIncluded)
{
  const std::string worldFile{"test/worlds/save_world.sdf"};
  this->LoadWorld(worldFile);
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  auto elem = std::make_shared<sdf::Element>();
  sdf::initFile("world.sdf", elem);

  this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_expand_include_tags()
      ->set_data(true);
  auto &modelGenConfig =
      *this->sdfGenConfig.mutable_override_entity_gen_configs();
  modelGenConfig["save_world::backpack2"]
      .mutable_expand_include_tags()
      ->set_data(false);
  sdf_generator::updateWorldElement(
      elem, this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);

  tinyxml2::XMLDocument originalSdfDoc;
  EXPECT_EQ(tinyxml2::XML_SUCCESS,
            originalSdfDoc.LoadFile(
                common::joinPaths(PROJECT_SOURCE_PATH, worldFile).c_str()));

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(elem->ToString("").c_str());

  // Compare elements from the original sdf xml and the generated xml
  auto origWorld = originalSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, origWorld);
  auto genWorld = genSdfDoc.RootElement();
  ASSERT_NE(nullptr, genWorld);

  // Check include tag
  auto origInclude = origWorld->FirstChildElement("include");
  ASSERT_NE(nullptr, origInclude);
  auto genInclude = genWorld->FirstChildElement("include");
  ASSERT_NE(nullptr, genInclude);
  {
    auto origUri = origInclude->FirstChildElement("uri");
    ASSERT_NE(nullptr, origUri);
    auto genUri = genInclude->FirstChildElement("uri");
    ASSERT_NE(nullptr, genUri);
    EXPECT_STREQ(origUri->GetText(), genUri->GetText());
  }

  tinyxml2::XMLElement *backpack2Model{nullptr};
  for (auto modelElem = genWorld->FirstChildElement("model"); modelElem;
       modelElem = modelElem->NextSiblingElement("model"))
  {
    if (std::string("backpack2") == modelElem->Attribute("name"))
    {
      backpack2Model = modelElem;
      break;
    }
  }
  // Check that backpack2 is not expanded
  EXPECT_EQ(nullptr, backpack2Model);
}

/////////////////////////////////////////////////
TEST_F(ElementUpdateFixture, WorldWithModelsUsingRelativeResourceURIs)
{
  const auto includeUri = std::string("file://") + PROJECT_SOURCE_PATH +
                          "/test/worlds/models/relative_resource_uri";

  std::string worldSdf = R"(
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="test_relative_resource">
    <include>
      <uri>)" + includeUri + R"(</uri>
    </include>
  </world>
</sdf>
  )";

  this->LoadWorldString(worldSdf);
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  EXPECT_EQ(1u, this->ecm.EntitiesByComponents(components::Model()).size());

  auto elem = std::make_shared<sdf::Element>();
  sdf::initFile("world.sdf", elem);
  this->sdfGenConfig.mutable_global_entity_gen_config()
      ->mutable_expand_include_tags()
      ->set_data(true);
  sdf_generator::updateWorldElement(
      elem, this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);

  auto link = elem->GetElement("model")->GetElement("link");

  for (auto elemType : {"visual", "collision"})
  {
    std::size_t testCount{0};
    for (auto testElem = link->GetElement(elemType); testElem;
         testElem = testElem->GetNextElement(elemType))
    {
      auto mesh = testElem->GetElement("geometry")->GetElement("mesh");
      ASSERT_TRUE(mesh->HasElement("uri"));
      auto uriStr = mesh->Get<std::string>("uri");
      EXPECT_EQ(includeUri + "/meshes/box.dae", uriStr);
      ++testCount;
    }
    EXPECT_EQ(2u, testCount);
  }
}

using GenerateWorldFixture = ElementUpdateFixture;
/////////////////////////////////////////////////
TEST_F(GenerateWorldFixture, ModelsInline)
{
  const std::string worldFile{"test/worlds/save_world.sdf"};
  this->LoadWorld(worldFile);
  Entity worldEntity = this->ecm.EntityByComponents(components::World());
  // Check with expandIncludeTags = true
  {
    this->sdfGenConfig.mutable_global_entity_gen_config()
        ->mutable_expand_include_tags()
        ->set_data(true);
    const std::optional<std::string> worldStr = sdf_generator::generateWorld(
        this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);
    ASSERT_TRUE(worldStr.has_value());
    // std::cout << "Generated world:" << std::endl;
    // std::cout << worldStr << std::endl;
    sdf::Root newRoot;
    newRoot.LoadSdfString(*worldStr);
    EXPECT_TRUE(isSubset(newRoot.Element(), this->root.Element()));
    EXPECT_TRUE(isSubset(this->root.Element(), newRoot.Element()));
  }
  // Check with expandIncludeTags = false
  {
    this->sdfGenConfig.mutable_global_entity_gen_config()
        ->mutable_expand_include_tags()
        ->set_data(false);
    const std::optional<std::string> worldStr = sdf_generator::generateWorld(
        this->ecm, worldEntity, this->includeUriMap, this->sdfGenConfig);
    ASSERT_TRUE(worldStr.has_value());
    // std::cout << "Generated world:" << std::endl;
    // std::cout << worldStr << std::endl;
    sdf::Root newRoot;
    newRoot.LoadSdfString(*worldStr);
    EXPECT_TRUE(isSubset(newRoot.Element(), this->root.Element()));
    EXPECT_TRUE(isSubset(this->root.Element(), newRoot.Element()));
  }
}

/////////////////////////////////////////////////
/// Main
int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  ::testing::AddGlobalTestEnvironment(
      new test::UniqueTestDirectoryEnv("sdf_gen_test_cache"));
  return RUN_ALL_TESTS();
}
