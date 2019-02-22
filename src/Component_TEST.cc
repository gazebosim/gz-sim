/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <memory>

#include <sdf/Element.hh>
#include <ignition/common/Console.hh>
#include <ignition/math/Inertial.hh>

#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
class ComponentTest : public ::testing::Test
{
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
  }
};

//////////////////////////////////////////////////
/// Test that using the default constructor of Component doesn't cause
/// problems when copying
TEST_F(ComponentTest, ComponentCanBeCopiedAfterDefaultCtor)
{
  // Use Component's default constructor
  auto comp = components::Name();

  // Test copy constructor and assignment
  components::Name compCopy(comp);
  comp = components::Name("test");

  // If it got here we have succeeded
  SUCCEED();
}

//////////////////////////////////////////////////
TEST_F(ComponentTest, DataByMove)
{
  // Create a custom component with shared_ptr data
  using CustomComponent =
      components::Component<std::shared_ptr<int>, class CustomComponentTag>;

  EntityComponentManager ecm;
  Entity entity = ecm.CreateEntity();

  auto data = std::make_shared<int>(1);
  // Copy data so we can check the use count after it has been moved
  auto dataCopy = data;

  EXPECT_EQ(2u, dataCopy.use_count());

  ecm.CreateComponent(entity, CustomComponent(std::move(data)));

  // If "data" was moved, the use cound should still be 2.
  EXPECT_EQ(2u, dataCopy.use_count());
}

// Class with externally defined stream operator
struct SimpleOperator {};
using CustomOperator = components::Component<SimpleOperator, class CustomTag>;

inline std::ostream &operator<<(std::ostream &_out, const SimpleOperator &)
{
  _out << "simple_operator";
  return _out;
}

// ostream operator for sdf::Element (not defined elsewhere)
// TODO(anyone) clang is not picking this up
inline std::ostream &operator<<(std::ostream &_out,
    const sdf::Element &_element)
{
  _out << _element.ToString("");
  return _out;
}

// ostream operator for math::Inertiald (not defined elsewhere)
// TODO(anyone) clang is not picking this up
inline std::ostream &operator<<(std::ostream &_out,
    const math::Inertiald &_inertial)
{
  _out << "Mass: " << _inertial.MassMatrix().Mass();
  return _out;
}

// Wrap existing class and give it a Serialize function
using InertialBase =
    components::Component<math::Inertiald, class InertialBaseTag>;
class InertialWrapper : public InertialBase
{
  public: InertialWrapper() : InertialBase()
  {
  }

  public: explicit InertialWrapper(const math::Inertiald &_data)
    : InertialBase(_data)
  {
  }

  public: void Serialize(std::ostream &_out) const override
  {
    _out << "Wrapper mass: " << this->Data().MassMatrix().Mass();
  }
};

//////////////////////////////////////////////////
TEST_F(ComponentTest, OStream)
{
  // Component with data which has stream operator
  {
    using Custom = components::Component<std::string, class CustomTag>;

    auto data = std::string("banana");
    Custom comp(data);

    std::ostringstream ostr;
    ostr << comp;
    EXPECT_EQ("banana", ostr.str());
  }

  // Component with data which doesn't have stream operator
  {
    struct Simple {};
    using Custom = components::Component<Simple, class CustomTag>;

    auto data = Simple();
    Custom comp(data);

    // Returns empty string and prints warning
    std::ostringstream ostr;
    ostr << comp;
    EXPECT_EQ("", ostr.str());
  }

  // Component with data which has custom stream operator
  {
    auto data = SimpleOperator();
    CustomOperator comp(data);

    std::ostringstream ostr;
    ostr << comp;
    EXPECT_EQ("simple_operator", ostr.str());
  }

  // Component with data which has custom stream operator
  {
    using Custom = components::Component<math::Inertiald, class CustomTag>;

    auto data = math::Inertiald();
    Custom comp(data);

    // TODO(anyone) Check why this passes with gcc but not with clang
    std::ostringstream ostr;
    ostr << comp;
    #if not defined (__clang__)
      EXPECT_EQ("Mass: 0", ostr.str());
    #else
      // clang is printing a warning and saying the operator<< is missing
      EXPECT_EQ("", ostr.str());
    #endif
  }

  // Component with data which has custom Serialize function
  {
    auto data = math::Inertiald();
    InertialWrapper comp(data);

    // TODO(anyone) Check why this passes with gcc but not with clang
    std::ostringstream ostr;
    ostr << comp;
    EXPECT_EQ("Wrapper mass: 0", ostr.str());
  }

  // Component with shared_ptr data which has stream operator
  {
    using Custom =
        components::Component<std::shared_ptr<int>, class CustomTag>;

    auto data = std::make_shared<int>(123);
    Custom comp(data);

    // Check the value is streamed, not the pointer address
    std::ostringstream ostr;
    ostr << comp;
    EXPECT_EQ("123", ostr.str());
  }

  // Component with shared_ptr data which doesn't have stream operator
  {
    struct Simple {};
    using Custom =
        components::Component<std::shared_ptr<Simple>, class CustomTag>;

    auto data = std::make_shared<Simple>();
    Custom comp(data);

    // Returns empty string and prints warning
    std::ostringstream ostr;
    ostr << comp;
    EXPECT_EQ("", ostr.str());
  }

  // Component with shared_ptr data which has custom stream operator
  {
    using Custom = components::Component<std::shared_ptr<SimpleOperator>,
        class CustomTag>;

    auto data = std::make_shared<SimpleOperator>();
    Custom comp(data);

    // Check the value is streamed, not the pointer address
    std::ostringstream ostr;
    ostr << comp;
    EXPECT_EQ("simple_operator", ostr.str());
  }

  // Component with shared_ptr sdf::Element, which has custom stream operator
  {
    using Custom = components::Component<std::shared_ptr<sdf::Element>,
        class CustomTag>;

    auto data = std::make_shared<sdf::Element>();
    data->SetName("element");
    data->AddAttribute("test", "string", "foo", false, "foo description");
    data->AddValue("string", "val", false, "val description");

    Custom comp(data);

    // TODO(anyone) Check why this passes with gcc but not with clang
    std::ostringstream ostr;
    ostr << comp;
    #if not defined (__clang__)
      EXPECT_EQ("<element test='foo'>val</element>\n", ostr.str());
    #else
      // clang is printing a warning and saying the operator<< is missing
      EXPECT_EQ("", ostr.str());
    #endif
  }

  // Component with shared_ptr math::Inertiald, which has custom stream operator
  {
    using Custom = components::Component<std::shared_ptr<math::Inertiald>,
        class CustomTag>;

    auto data = std::make_shared<math::Inertiald>();
    Custom comp(data);

    // TODO(anyone) Check why this passes with gcc but not with clang
    std::ostringstream ostr;
    ostr << comp;
    #if not defined (__clang__)
      EXPECT_EQ("Mass: 0", ostr.str());
    #else
      // clang is printing a warning and saying the operator<< is missing
      EXPECT_EQ("", ostr.str());
    #endif
  }
}
