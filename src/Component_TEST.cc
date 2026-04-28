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
#include <gz/msgs/int32.pb.h>
#include <gz/utils/ExtraTestMacros.hh>

#include <memory>

#include <sdf/Element.hh>
#include <gz/common/Console.hh>
#include <gz/math/Inertial.hh>

#include "gz/sim/components/Component.hh"
#include "gz/sim/components/Serialization.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/EntityComponentManager.hh"

#include "../test/helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

// Class with externally defined stream operator
struct SimpleOperator
{
  int data{100};
};

struct Simple {};

using CustomComponent =
      components::Component<std::shared_ptr<int>, class CustomComponentTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.MyCustom", CustomComponent)

using CustomString = components::Component<std::string, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomString", CustomString)

using CustomSimple = components::Component<Simple, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomSimple", CustomSimple)

using CustomOperator = components::Component<SimpleOperator, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomOperator", CustomOperator)

using CustomInertial = components::Component<math::Inertiald, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomInertial", CustomInertial)

using CustomSharedInt = components::Component<std::shared_ptr<int>, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomSharedInt", CustomSharedInt)

using CustomSharedSimple = components::Component<std::shared_ptr<Simple>, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomSharedSimple", CustomSharedSimple)

using CustomSharedSimpleOperator = components::Component<std::shared_ptr<SimpleOperator>, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomSharedSimpleOperator", CustomSharedSimpleOperator)

using CustomSharedSdf = components::Component<std::shared_ptr<sdf::Element>, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomSharedSdf", CustomSharedSdf)

using CustomSharedInertial = components::Component<std::shared_ptr<math::Inertiald>, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomSharedInertial", CustomSharedInertial)

using CustomMsg = components::Component<msgs::Int32, class CustomTag, serializers::MsgSerializer>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomMsg", CustomMsg)

//////////////////////////////////////////////////
class ComponentTest : public InternalFixture<::testing::Test>
{
  protected: void SetUp() override
  {
    InternalFixture::SetUp();
    common::setenv("GZ_DEBUG_COMPONENT_FACTORY", "true");
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
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(ComponentTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(DataByMove))
{
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
namespace gz
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
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.InertialBase", InertialBase)
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

  public: std::unique_ptr<BaseComponent> Clone() const override
  {
    return nullptr;
  }
};

//////////////////////////////////////////////////
TEST_F(ComponentTest, OStream)
{
  // Component with data which has stream operator
  {
    using Custom = CustomString;

    auto data = std::string("banana");
    Custom comp(data);

    std::ostringstream ostr;
    comp.Serialize(ostr);
    EXPECT_EQ("banana", ostr.str());
  }

  // Component with data which doesn't have stream operator
  {
    using Custom = CustomSimple;

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
    using Custom = CustomInertial;

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
    using Custom = CustomSharedInt;

    auto data = std::make_shared<int>(123);
    Custom comp(data);

    // Check the value is streamed, not the pointer address
    std::ostringstream ostr;
    comp.Serialize(ostr);
    EXPECT_EQ("123", ostr.str());
  }

  // Component with shared_ptr data which doesn't have stream operator
  {
    using Custom = CustomSharedSimple;

    auto data = std::make_shared<Simple>();
    Custom comp(data);

    // Returns empty string and prints warning
    std::ostringstream ostr;
    comp.Serialize(ostr);
    EXPECT_EQ("", ostr.str());
  }

  // Component with shared_ptr data which has custom stream operator
  {
    using Custom = CustomSharedSimpleOperator;

    auto data = std::make_shared<SimpleOperator>();
    Custom comp(data);

    // Check the value is streamed, not the pointer address
    std::ostringstream ostr;
    comp.Serialize(ostr);
    EXPECT_EQ("simple_operator", ostr.str());
  }

  // Component with shared_ptr sdf::Element, which has custom stream operator
  {
    using Custom = CustomSharedSdf;

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
    using Custom = CustomSharedInertial;

    auto data = std::make_shared<math::Inertiald>();
    Custom comp(data);

    std::ostringstream ostr;
    comp.Serialize(ostr);
    EXPECT_EQ("Mass: 0", ostr.str());
  }

  // Component with a msgs type that gets serialized by the default
  // serializer
  {
    using Custom = CustomMsg;

    msgs::Int32 data;
    data.set_data(331);
    Custom comp(data);

    std::ostringstream ostr;
    comp.Serialize(ostr);

    msgs::Int32 dataExpected;
    EXPECT_TRUE(dataExpected.ParseFromString(ostr.str()));
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
    using Custom = CustomString;

    std::istringstream istr("banana");
    Custom comp;
    comp.Deserialize(istr);
    EXPECT_EQ("banana", comp.Data());
  }

  // Component with data which doesn't have stream operator
  {
    using Custom = CustomSimple;

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
    using Custom = CustomInertial;

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
    using Custom = CustomSharedInt;

    auto data = std::make_shared<int>(123);
    Custom comp(data);

    // Check the value is streamed, not the pointer address
    std::istringstream istr("456");
    comp.Deserialize(istr);
    EXPECT_EQ(456, *comp.Data());
  }

  // Component with shared_ptr data which doesn't have stream operator
  {
    using Custom = CustomSharedSimple;

    auto data = std::make_shared<Simple>();
    Custom comp(data);

    // Prints warning and doesn't modify the component
    std::istringstream istr("ignored");
    comp.Deserialize(istr);
  }

  // Component with shared_ptr data which has custom stream operator
  {
    using Custom = CustomSharedSimpleOperator;

    auto data = std::make_shared<SimpleOperator>();
    Custom comp(data);

    // Check the value is changed, not the pointer address
    std::istringstream istr("not used");
    comp.Deserialize(istr);
    EXPECT_EQ(456, comp.Data()->data);
  }

  // Component with shared_ptr sdf::Element, which has custom stream operator
  {
    using Custom = CustomSharedSdf;

    auto data = std::make_shared<sdf::Element>();
    Custom comp(data);

    std::istringstream istr("not used");
    comp.Deserialize(istr);
    EXPECT_EQ("<new_name/>\n", comp.Data()->ToString(""));
  }

  // Component with shared_ptr math::Inertiald, which has custom stream operator
  {
    using Custom = CustomSharedInertial;

    auto data = std::make_shared<math::Inertiald>();
    Custom comp(data);

    std::istringstream istr("not used");
    comp.Deserialize(istr);
    EXPECT_DOUBLE_EQ(200, comp.Data()->MassMatrix().Mass());
  }

  // Component with a msgs type that gets deserialized by the message
  // deserializer
  {
    using Custom = CustomMsg;

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

namespace test_components
{
  using ConstexprComp = gz::sim::components::Component<int, class ConstexprTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.ConstexprComp", ConstexprComp)
}

//////////////////////////////////////////////////
TEST_F(ComponentTest, TypeId)
{
  // Constexpr TypeId
  {
    using ConstexprComp = test_components::ConstexprComp;
    EXPECT_EQ(ConstexprComp::typeId,
        common::hash64("gz_sim_components.ConstexprComp"));
  }

  // Pre-registered component
  {
    EXPECT_EQ(components::Name::typeId,
              common::hash64("gz_sim_components.Name"));
  }
}

using CustomInt = components::Component<int, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomInt", CustomInt)

using CustomNoData = components::Component<components::NoData, class CustomTag>;
GZ_SIM_REGISTER_COMPONENT("gz_sim_components.CustomNoData", CustomNoData)

//////////////////////////////////////////////////
TEST_F(ComponentTest, TypeName)
{
  // Component with data
  {
    CustomInt::typeName = "123456";

    CustomInt comp;

    EXPECT_EQ("123456", comp.typeName);
  }

  // components::Component<double

  // Component without data
  {
    CustomNoData::typeName = "123456";

    CustomNoData comp;

    EXPECT_EQ("123456", comp.typeName);
  }
}

//////////////////////////////////////////////////
TEST_F(ComponentTest, Clone)
{
  // Component with data
  {
    // create a component and a clone of it. The clone should initially have the
    // same data as the original component
    CustomInt comp(5);
    auto clonedComp = comp.Clone();
    auto derivedClone = static_cast<CustomInt *>(clonedComp.get());
    EXPECT_EQ(comp, *derivedClone);

    // modify the data of the cloned component, and make sure that only the
    // cloned component is modified, not the original component
    derivedClone->Data() = 10;
    EXPECT_NE(comp, *derivedClone);
    EXPECT_EQ(5, comp.Data());
    EXPECT_EQ(10, derivedClone->Data());
  }

  // Component without data
  {
    CustomNoData comp;
    auto clonedComp = comp.Clone();
    auto derivedClone = static_cast<CustomNoData *>(clonedComp.get());

    // since this component has no data, we cannot do the same check as we did
    // above for a component with data. However, we can make sure that the
    // pointers for the components are different
    EXPECT_NE(&comp, derivedClone);
  }
}
