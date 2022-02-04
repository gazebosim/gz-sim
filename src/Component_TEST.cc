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
#include <ignition/msgs/int32.pb.h>

#include <memory>

#include <sdf/Element.hh>
#include <ignition/common/Console.hh>
#include <ignition/math/Inertial.hh>

#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/components/Serialization.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "../test/helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
class ComponentTest : public InternalFixture<::testing::Test>
{
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
  auto factory = components::Factory::Instance();

  // Create a custom component with shared_ptr data
  using CustomComponent =
      components::Component<std::shared_ptr<int>, class CustomComponentTag>;
  factory->Register<CustomComponent>("ign_gazebo_components.MyCustom",
     new components::ComponentDescriptor<CustomComponent>(),
     new components::StorageDescriptor<CustomComponent>());

  EntityComponentManager ecm;
  Entity entity = ecm.CreateEntity();

  auto data = std::make_shared<int>(1);
  // Copy data so we can check the use count after it has been moved
  auto dataCopy = data;

  EXPECT_EQ(2u, dataCopy.use_count());

  ecm.CreateComponent(entity, CustomComponent(std::move(data)));

  // If "data" was moved, the use count should still be 2.
  EXPECT_EQ(2u, dataCopy.use_count());
}

// Class with externally defined stream operator
struct SimpleOperator
{
  int data{100};
};
using CustomOperator = components::Component<SimpleOperator, class CustomTag>;

inline std::ostream &operator<<(std::ostream &_out, const SimpleOperator &)
{
  _out << "simple_operator";
  return _out;
}

inline std::istream &operator>>(std::istream &_in, SimpleOperator &_op)
{
  _op.data = 456;
  return _in;
}

// ostream operator for sdf::Element (not defined elsewhere)
// Note: Must be defined in the correct namespace or clang refuses to find it.
namespace sdf
{
inline std::ostream &operator<<(std::ostream &_out,
    const sdf::Element &_element)
{
  _out << _element.ToString("");
  return _out;
}
inline std::istream &operator>>(std::istream &_in, sdf::Element &_element)
{
  _element.SetName("new_name");
  return _in;
}
}

// ostream operator for math::Inertiald (not defined elsewhere)
// Note: Must be defined in the correct namespace or clang refuses to find it.
namespace ignition
{
namespace math
{
inline std::ostream &operator<<(std::ostream &_out, const Inertiald &_inertial)
{
  _out << "Mass: " << _inertial.MassMatrix().Mass();
  return _out;
}
inline std::istream &operator>>(std::istream &_in, Inertiald &_inertial)
{
  auto mat = _inertial.MassMatrix();
  mat.SetMass(200);
  _inertial.SetMassMatrix(mat);
  return _in;
}
}
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

  public: void Deserialize(std::istream &) override
  {
    auto mat = this->Data().MassMatrix();
    mat.SetMass(2000);
    this->Data().SetMassMatrix(mat);
  }
};

// Component without De/Serialize
class NoSerialize : public components::BaseComponent
{
  public: ComponentTypeId TypeId() const override
  {
    return 0;
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
    comp.Serialize(ostr);
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
    comp.Serialize(ostr);
    EXPECT_EQ("", ostr.str());
  }

  // Component with data which has custom stream operator
  {
    auto data = SimpleOperator();
    CustomOperator comp(data);

    std::ostringstream ostr;
    comp.Serialize(ostr);
    EXPECT_EQ("simple_operator", ostr.str());
  }

  // Component with data which has custom stream operator
  {
    using Custom = components::Component<math::Inertiald, class CustomTag>;

    auto data = math::Inertiald();
    auto comp = new Custom(data);

    std::ostringstream ostr;
    comp->Serialize(ostr);
    EXPECT_EQ("Mass: 0", ostr.str());

    // Serializable from base class
    auto compBase = dynamic_cast<components::BaseComponent *>(comp);
    std::ostringstream ostrBase;
    compBase->Serialize(ostrBase);
    EXPECT_EQ("Mass: 0", ostrBase.str());
  }

  // Component with data which has custom Serialize function
  {
    auto data = math::Inertiald();
    auto comp = new InertialWrapper(data);

    std::ostringstream ostr;
    comp->Serialize(ostr);
    EXPECT_EQ("Wrapper mass: 0", ostr.str());

    // Serializable from base class
    auto compBase = dynamic_cast<components::BaseComponent *>(comp);
    std::ostringstream ostrBase;
    compBase->Serialize(ostrBase);
    EXPECT_EQ("Wrapper mass: 0", ostrBase.str());
  }

  // Component with shared_ptr data which has stream operator
  {
    using Custom =
        components::Component<std::shared_ptr<int>, class CustomTag>;

    auto data = std::make_shared<int>(123);
    Custom comp(data);

    // Check the value is streamed, not the pointer address
    std::ostringstream ostr;
    comp.Serialize(ostr);
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
    comp.Serialize(ostr);
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
    comp.Serialize(ostr);
    EXPECT_EQ("simple_operator", ostr.str());
  }

  // Component with shared_ptr sdf::Element, which has custom stream operator
  {
    using Custom = components::Component<std::shared_ptr<sdf::Element>,
        class CustomTag>;

    auto data = std::make_shared<sdf::Element>();
    data->SetName("element");
    data->AddAttribute("test", "string", "foo", true, "foo description");
    data->AddValue("string", "val", false, "val description");

    Custom comp(data);

    std::ostringstream ostr;
    comp.Serialize(ostr);
    EXPECT_EQ("<element test='foo'>val</element>\n", ostr.str());
  }

  // Component with shared_ptr math::Inertiald, which has custom stream operator
  {
    using Custom = components::Component<std::shared_ptr<math::Inertiald>,
        class CustomTag>;

    auto data = std::make_shared<math::Inertiald>();
    Custom comp(data);

    std::ostringstream ostr;
    comp.Serialize(ostr);
    EXPECT_EQ("Mass: 0", ostr.str());
  }

  // Component with a ignition::msgs type that gets serialized by the default
  // serializer
  {
    using Custom = components::Component<msgs::Int32, class CustomTag,
        serializers::MsgSerializer>;

    msgs::Int32 data;
    data.set_data(331);
    Custom comp(data);

    std::ostringstream ostr;
    comp.Serialize(ostr);

    msgs::Int32 dataExpected;
    dataExpected.ParseFromString(ostr.str());
    EXPECT_EQ(331, dataExpected.data());
  }

  // Component without Serialize function
  {
    NoSerialize comp;

    std::ostringstream ostr;
    comp.Serialize(ostr);
    EXPECT_EQ("", ostr.str());
  }
}

//////////////////////////////////////////////////
TEST_F(ComponentTest, IStream)
{
  // Component with data which has stream operator
  {
    using Custom = components::Component<std::string, class CustomTag>;

    std::istringstream istr("banana");
    Custom comp;
    comp.Deserialize(istr);
    EXPECT_EQ("banana", comp.Data());
  }

  // Component with data which doesn't have stream operator
  {
    struct Simple {};
    using Custom = components::Component<Simple, class CustomTag>;

    // Prints warning and doesn't modify the component
    std::istringstream istr("banana");
    Custom comp;
    comp.Deserialize(istr);
  }

  // Component with data which has custom stream operator
  {
    auto data = SimpleOperator();
    CustomOperator comp(data);

    std::istringstream istr("not used");
    comp.Deserialize(istr);
    EXPECT_EQ(456, comp.Data().data);
  }

  // Component with data which has custom stream operator
  {
    using Custom = components::Component<math::Inertiald, class CustomTag>;

    auto data = math::Inertiald();
    Custom comp(data);

    std::istringstream istr("not used");
    comp.Deserialize(istr);
    EXPECT_DOUBLE_EQ(200, comp.Data().MassMatrix().Mass());
  }

  // Component with data which has custom Deserialize function
  {
    auto data = math::Inertiald();
    InertialWrapper comp(data);

    std::istringstream istr("not used");
    comp.Deserialize(istr);
    EXPECT_DOUBLE_EQ(2000, comp.Data().MassMatrix().Mass());
  }

  // Component with shared_ptr data which has stream operator
  {
    using Custom =
        components::Component<std::shared_ptr<int>, class CustomTag>;

    auto data = std::make_shared<int>(123);
    Custom comp(data);

    // Check the value is streamed, not the pointer address
    std::istringstream istr("456");
    comp.Deserialize(istr);
    EXPECT_EQ(456, *comp.Data());
  }

  // Component with shared_ptr data which doesn't have stream operator
  {
    struct Simple {};
    using Custom =
        components::Component<std::shared_ptr<Simple>, class CustomTag>;

    auto data = std::make_shared<Simple>();
    Custom comp(data);

    // Prints warning and doesn't modify the component
    std::istringstream istr("ignored");
    comp.Deserialize(istr);
  }

  // Component with shared_ptr data which has custom stream operator
  {
    using Custom = components::Component<std::shared_ptr<SimpleOperator>,
        class CustomTag>;

    auto data = std::make_shared<SimpleOperator>();
    Custom comp(data);

    // Check the value is changed, not the pointer address
    std::istringstream istr("not used");
    comp.Deserialize(istr);
    EXPECT_EQ(456, comp.Data()->data);
  }

  // Component with shared_ptr sdf::Element, which has custom stream operator
  {
    using Custom = components::Component<std::shared_ptr<sdf::Element>,
        class CustomTag>;

    auto data = std::make_shared<sdf::Element>();
    Custom comp(data);

    std::istringstream istr("not used");
    comp.Deserialize(istr);
    EXPECT_EQ("<new_name/>\n", comp.Data()->ToString(""));
  }

  // Component with shared_ptr math::Inertiald, which has custom stream operator
  {
    using Custom = components::Component<std::shared_ptr<math::Inertiald>,
        class CustomTag>;

    auto data = std::make_shared<math::Inertiald>();
    Custom comp(data);

    std::istringstream istr("not used");
    comp.Deserialize(istr);
    EXPECT_DOUBLE_EQ(200, comp.Data()->MassMatrix().Mass());
  }

  // Component with a ignition::msgs type that gets deserialized by the message
  // deserializer
  {
    using Custom = components::Component<msgs::Int32, class CustomTag,
        serializers::MsgSerializer>;

    msgs::Int32 data;
    data.set_data(482);

    std::istringstream istr(data.SerializeAsString());

    Custom comp;
    comp.Deserialize(istr);

    EXPECT_EQ(482, comp.Data().data());
  }

  // Component without Deserialize function
  {
    NoSerialize comp;

    std::istringstream istr("not used");
    comp.Deserialize(istr);
  }
}

//////////////////////////////////////////////////
TEST_F(ComponentTest, TypeId)
{
  // Component with data
  {
    using Custom = components::Component<int, class CustomTag>;
    Custom::typeId = 123456;

    Custom comp;

    EXPECT_EQ(ComponentTypeId(123456), comp.TypeId());
  }

  // Component without data
  {
    using Custom = components::Component<components::NoData, class CustomTag>;
    Custom::typeId = 123456;

    Custom comp;

    EXPECT_EQ(ComponentTypeId(123456), comp.TypeId());
  }
}

//////////////////////////////////////////////////
TEST_F(ComponentTest, TypeName)
{
  // Component with data
  {
    using Custom = components::Component<int, class CustomTag>;
    Custom::typeName = "123456";

    Custom comp;

    EXPECT_EQ("123456", comp.typeName);
  }

  // Component without data
  {
    using Custom = components::Component<components::NoData, class CustomTag>;
    Custom::typeName = "123456";

    Custom comp;

    EXPECT_EQ("123456", comp.typeName);
  }
}
