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
#ifndef IGNITION_GAZEBO_COMPONENTS_COMPONENT_HH_
#define IGNITION_GAZEBO_COMPONENTS_COMPONENT_HH_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <ignition/common/Console.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  // Forward declarations.
  template<typename DataType> class ComponentPrivate;

  /// \brief Convenient type to be used by components that don't wrap any data.
  /// I.e. they act as tags and their presence is enough to infer something
  /// about the entity.
  using NoData = std::add_lvalue_reference<void>;

  /// \brief Base class for all components.
  class BaseComponent
  {
    /// \brief Default constructor.
    public: BaseComponent() = default;

    /// \brief Default destructor.
    public: virtual ~BaseComponent() = default;

    /// \brief Returns the unique name for the component's type.
    /// The name is manually chosen during the Factory registration.
    public: virtual std::string TypeName() const = 0;

    /// \brief Returns the unique ID for the component's type.
    /// The ID is derived from the name that is manually chosen during the
    /// Factory registration and is guaranteed to be the same across compilers
    /// and runs.
    public: virtual ComponentTypeId TypeId() const = 0;
  };

  /// \brief A component type that wraps any data type. The intention is for
  /// this class to be used to create simple components while avoiding a lot of
  /// boilerplate code. The Identifier must be a unique type so that type
  /// aliases can be used to create new components. However the type does not
  /// need to be defined anywhere
  /// eg.
  ///     using Static = Component<bool, class StaticTag>;
  ///
  /// Note, however, that this scheme does not have a mechanism to stop someone
  /// accidentally defining another component that wraps a bool as such:
  ///     using AnotherComp = Component<bool, class StaticTag>;
  /// In this case, Static and AnotherComp are exactly the same types and would
  /// not be differentiable by the EntityComponentManager.
  ///
  /// \tparam DataType Type of the data being wrapped by this component.
  /// \tparam Identifier Unique identifier for the component class, to avoid
  /// collision.
  template <typename DataType, typename Identifier>
  class Component: public BaseComponent
  {
    /// \brief Default constructor
    public: Component();

    /// \brief Constructor
    /// \param[in] _data Data to copy
    public: explicit Component(const DataType &_data);

    /// \brief Constructor data to be moved
    /// \param[in] _data Data to moved
    public: explicit Component(DataType &&_data);

    /// \brief Copy Constructor
    /// \param[in] _component Component component to copy.
    public: Component(const Component &_component);

    /// \brief Move Constructor
    /// \param[in] _component Component component to move.
    public: Component(Component &&_component) noexcept = default;

    /// \brief Destructor.
    public: ~Component() override = default;

    /// \brief Move assignment operator.
    /// \param[in] _component Component component to move.
    /// \return Reference to this.
    public: Component &operator=(
                Component &&_component) noexcept = default;

    /// \brief Copy assignment operator.
    /// \param[in] _component Component component to copy.
    /// \return Reference to this.
    public: Component &operator=(const Component &_component);

    /// \brief Equality operator.
    /// \param[in] _component Component to compare to.
    /// \return True if equal.
    public: bool operator==(const Component &_component) const;

    /// \brief Inequality operator.
    /// \param[in] _component Component to compare to.
    /// \return True if different.
    public: bool operator!=(const Component &_component) const;

    // Documentation inherited
    public: std::string TypeName() const override;

    // Documentation inherited
    public: ComponentTypeId TypeId() const override;

    /// \brief Get the component data.
    /// \return The actual component information.
    public: const DataType &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<ComponentPrivate<DataType>> dataPtr;

    /// \brief Unique name for this component type. This is set through the
    /// Factory registration.
    public: inline static std::string typeName{""};

    /// \brief Unique ID for this component type. This is set through the
    /// Factory registration.
    public: inline static ComponentTypeId typeId{0};
  };

  /// \brief Specialization for components that don't wrap any data.
  /// This class to be used to create simple components that represent
  /// just a "tag", while avoiding a lot of boilerplate code. The Identifier
  /// must be a unique type so that type aliases can be used to create new
  /// components. However the type does not need to be defined anywhere eg.
  ///
  ///     using Joint = Component<NoData, class JointTag>;
  ///
  template <typename Identifier>
  class Component<NoData, Identifier> : public BaseComponent
  {
    // Documentation inherited
    public: bool operator==(const Component<NoData, Identifier> &) const;

    // Documentation inherited
    public: bool operator!=(const Component<NoData, Identifier> &) const;

    // Documentation inherited
    public: std::string TypeName() const override;

    // Documentation inherited
    public: uint64_t TypeId() const override;

    /// \brief Unique name for this component type. This is set through the
    /// Factory registration.
    public: inline static std::string typeName{""};

    /// \brief Unique ID for this component type. This is set through the
    /// Factory registration.
    public: inline static ComponentTypeId typeId{0};
  };

  template <typename DataType>
  class ComponentPrivate
  {
    /// \brief Default constructor
    public: ComponentPrivate() = default;

    /// \brief Constructor.
    /// \param[in] _component Component data.
    public: explicit ComponentPrivate(DataType _data)
            : data(std::move(_data))
    {
    }

    /// \brief The data being wrapped.
    public: DataType data;
  };

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  Component<DataType, Identifier>::Component()
    : dataPtr(std::make_unique<ComponentPrivate<DataType>>())
  {
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  Component<DataType, Identifier>::Component(const DataType &_data)
    : dataPtr(std::make_unique<ComponentPrivate<DataType>>(_data))
  {
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  Component<DataType, Identifier>::Component(DataType &&_data)
    : dataPtr(std::make_unique<ComponentPrivate<DataType>>(std::move(_data)))
  {
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  Component<DataType, Identifier>::Component(
      const Component<DataType, Identifier> &_component)
      : dataPtr(std::make_unique<ComponentPrivate<DataType>>(
            _component.Data()))
  {
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  const DataType &Component<DataType, Identifier>::Data() const
  {
    return this->dataPtr->data;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  Component<DataType, Identifier> &Component<DataType, Identifier>::
  operator=(const Component<DataType, Identifier> &_component)
  {
    this->dataPtr->data = _component.Data();
    return *this;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  bool Component<DataType, Identifier>::
  operator==(const Component<DataType, Identifier> &_component) const
  {
    return this->dataPtr->data == _component.Data();
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  bool Component<DataType, Identifier>::
  operator!=(const Component<DataType, Identifier> &_component) const
  {
    return this->dataPtr->data != _component.Data();
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  std::string Component<DataType, Identifier>::TypeName() const
  {
    return typeName;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  ComponentTypeId Component<DataType, Identifier>::TypeId() const
  {
    return typeId;
  }

  //////////////////////////////////////////////////
  template <typename Identifier>
  bool Component<NoData, Identifier>::operator==(
      const Component<NoData, Identifier> &) const
  {
    return true;
  }

  //////////////////////////////////////////////////
  template <typename Identifier>
  bool Component<NoData, Identifier>::operator!=(
      const Component<NoData, Identifier> &) const
  {
    return false;
  }

  //////////////////////////////////////////////////
  template <typename Identifier>
  std::string Component<NoData, Identifier>::TypeName() const
  {
    return typeName;
  }

  //////////////////////////////////////////////////
  template <typename Identifier>
  ComponentTypeId Component<NoData, Identifier>::TypeId() const
  {
    return typeId;
  }
}
}
}
}
#endif
