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

#include <gtest/gtest.h>

#include <cstddef>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"

/////////////////////////////////////////////////
TEST(ModelTest, Constructor)
{
  gz::sim::Model modelNull;
  EXPECT_EQ(gz::sim::kNullEntity, modelNull.Entity());

  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  EXPECT_EQ(id, model.Entity());
}

/////////////////////////////////////////////////
TEST(ModelTest, CopyConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  // Marked nolint because we are specifically testing copy
  // constructor here (clang wants unnecessary copies removed)
  gz::sim::Model modelCopy(model); // NOLINT
  EXPECT_EQ(model.Entity(), modelCopy.Entity());
}

/////////////////////////////////////////////////
TEST(ModelTest, CopyAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  gz::sim::Model modelCopy;
  modelCopy = model;
  EXPECT_EQ(model.Entity(), modelCopy.Entity());
}

/////////////////////////////////////////////////
TEST(ModelTest, MoveConstructor)
{
  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  gz::sim::Model modelMoved(std::move(model));
  EXPECT_EQ(id, modelMoved.Entity());
}

/////////////////////////////////////////////////
TEST(ModelTest, MoveAssignmentOperator)
{
  gz::sim::Entity id(3);
  gz::sim::Model model(id);

  gz::sim::Model modelMoved;
  modelMoved = std::move(model);
  EXPECT_EQ(id, modelMoved.Entity());
}

/////////////////////////////////////////////////
TEST(ModelTest, Links)
{
  // modelA
  //  - linkAA
  //  - linkAB
  //  - modelB
  //    - linkBA
  //
  // modelC

  gz::sim::EntityComponentManager ecm;

  // Model A
  auto modelAEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelAEntity, gz::sim::components::Model());
  ecm.CreateComponent(modelAEntity,
      gz::sim::components::Name("modelA_name"));

  // Link AA - Child of Model A
  auto linkAAEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkAAEntity, gz::sim::components::Link());
  ecm.CreateComponent(linkAAEntity,
      gz::sim::components::Name("linkAA_name"));
  ecm.CreateComponent(linkAAEntity,
      gz::sim::components::ParentEntity(modelAEntity));

  // Link AB - Child of Model A
  auto linkABEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkABEntity, gz::sim::components::Link());
  ecm.CreateComponent(linkABEntity,
      gz::sim::components::Name("linkAB_name"));
  ecm.CreateComponent(linkABEntity,
      gz::sim::components::ParentEntity(modelAEntity));

  // Model B - Child of Model A
  auto modelBEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelBEntity, gz::sim::components::Model());
  ecm.CreateComponent(modelBEntity,
      gz::sim::components::Name("modelB_name"));
  ecm.CreateComponent(modelBEntity,
      gz::sim::components::ParentEntity(modelAEntity));

  // Link BA - Child of Model B
  auto linkBAEntity = ecm.CreateEntity();
  ecm.CreateComponent(linkBAEntity, gz::sim::components::Link());
  ecm.CreateComponent(linkBAEntity,
      gz::sim::components::Name("linkBA_name"));
  ecm.CreateComponent(linkBAEntity,
      gz::sim::components::ParentEntity(modelBEntity));

  // Model C
  auto modelCEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelCEntity, gz::sim::components::Model());
  ecm.CreateComponent(modelCEntity,
      gz::sim::components::Name("modelC_name"));

  std::size_t foundLinks = 0;

  gz::sim::Model modelA(modelAEntity);
  auto links = modelA.Links(ecm);
  EXPECT_EQ(2u, links.size());
  for (const auto &link : links)
  {
    if (link == linkAAEntity || link == linkABEntity)
      foundLinks++;
  }
  EXPECT_EQ(foundLinks, links.size());

  gz::sim::Model modelB(modelBEntity);
  links = modelB.Links(ecm);
  ASSERT_EQ(1u, links.size());
  EXPECT_EQ(linkBAEntity, links[0]);

  gz::sim::Model modelC(modelCEntity);
  EXPECT_EQ(0u, modelC.Links(ecm).size());
}

/////////////////////////////////////////////////
TEST(ModelTest, Models)
{
  // modelA
  //  - modelB
  //  - modelC
  //    - modelD

  gz::sim::EntityComponentManager ecm;

  // Model A
  auto modelAEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelAEntity, gz::sim::components::Model());
  ecm.CreateComponent(modelAEntity,
      gz::sim::components::Name("modelA_name"));

  // Model B - Child of Model A
  auto modelBEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelBEntity, gz::sim::components::Model());
  ecm.CreateComponent(modelBEntity,
      gz::sim::components::Name("modelB_name"));
  ecm.CreateComponent(modelBEntity,
      gz::sim::components::ParentEntity(modelAEntity));

  // Model C - Child of Model A
  auto modelCEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelCEntity, gz::sim::components::Model());
  ecm.CreateComponent(modelCEntity,
      gz::sim::components::Name("modelC_name"));
  ecm.CreateComponent(modelCEntity,
      gz::sim::components::ParentEntity(modelAEntity));

  // Model D - Child of Model C
  auto modelDEntity = ecm.CreateEntity();
  ecm.CreateComponent(modelDEntity, gz::sim::components::Model());
  ecm.CreateComponent(modelDEntity,
      gz::sim::components::Name("modelD_name"));
  ecm.CreateComponent(modelDEntity,
      gz::sim::components::ParentEntity(modelCEntity));

  std::size_t foundModels = 0;

  gz::sim::Model modelA(modelAEntity);
  auto models = modelA.Models(ecm);
  EXPECT_EQ(2u, models.size());
  for (const auto &model : models)
  {
    if (model == modelBEntity || model == modelCEntity)
      foundModels++;
  }
  EXPECT_EQ(foundModels, models.size());

  gz::sim::Model modelB(modelBEntity);
  EXPECT_EQ(0u, modelB.Models(ecm).size());

  gz::sim::Model modelC(modelCEntity);
  models = modelC.Models(ecm);
  ASSERT_EQ(1u, models.size());
  EXPECT_EQ(modelDEntity, models[0]);

  gz::sim::Model modelD(modelDEntity);
  EXPECT_EQ(0u, modelD.Models(ecm).size());
}
