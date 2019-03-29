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
#include <sstream>
#include <utility>

#include <ignition/common/Console.hh>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>
#include <ignition/gazebo/Types.hh>

/// \brief Helper trait to determine if a type is shared_ptr or not
template<typename T> struct IsSharedPtr:
  std::false_type
{
};

/// \brief Helper trait to determine if a type is shared_ptr or not
template<typename T> struct IsSharedPtr<std::shared_ptr<T>>:
  std::true_type
{
};

/// \brief Helper template to call stream operators only on types that support
/// them.
/// This version is called for types that have operator<<
/// \tparam DataType Type on which the operator will be called.
/// \tparam Identifier Unique identifier for the component class.
/// \tparam Stream Type used to check if component has operator<<
/// \param[in] _out Out stream.
/// \param[in] _data Data to be serialized.
template<typename DataType, typename Identifier,
  typename Stream =
  decltype(std::declval<std::ostream &>() << std::declval<DataType const &>()),
  typename std::enable_if<
    !IsSharedPtr<DataType>::value &&
    std::is_convertible<Stream, std::ostream &>::value,
    int>::type = 0>
std::ostream &toStream(std::ostream &_out, DataType const &_data)
{
  _out << _data;
  return _out;
}

/// \brief Helper template to call stream operators only on types that support
/// them.
/// This version is called for types that are pointers to types that have
/// operator<<
/// \tparam DataType Type on which the operator will be called.
/// \tparam Identifier Unique identifier for the component class.
/// \tparam Stream Type used to check if component has operator<<
/// \param[in] _out Out stream.
/// \param[in] _data Data to be serialized.
template<typename DataType, typename Identifier,
  typename Stream =
  decltype(std::declval<std::ostream &>() << std::declval<
    typename DataType::element_type const &>()),
  typename std::enable_if<
    IsSharedPtr<DataType>::value &&
    std::is_convertible<Stream, std::ostream &>::value,
    int>::type = 0>
std::ostream &toStream(std::ostream &_out, DataType const &_data)
{
  _out << *_data;
  return _out;
}

/// \brief Helper template to call stream operators only on types that support
/// them.
/// This version is called for types that don't have operator<<
/// \tparam DataType Type on which the operator will be called.
/// \tparam Identifier Unique identifier for the component class.
/// \tparam Ignored All other template parameters are ignored.
/// \param[in] _out Out stream.
/// \param[in] _data Data to be serialized.
template<typename DataType, typename Identifier, typename... Ignored>
std::ostream &toStream(std::ostream &_out, DataType const &,
    Ignored const &..., ...)
{
  static bool warned{false};
  if (!warned)
  {
    ignwarn << "Trying to serialize component with data type ["
            << typeid(DataType).name() << "], which doesn't have "
            << "`operator<<`. Component will not be serialized." << std::endl;
    warned = true;
  }
  return _out;
}

/// \brief Helper template to call extract operators only on types that support
/// them.
/// This version is called for types that have operator>>
/// \tparam DataType Type on which the operator will be called.
/// \tparam Identifier Unique identifier for the component class.
/// \tparam Stream Type used to check if component has operator>>
/// \param[in] _in In stream.
/// \param[in] _data Data resulting from deserialization.
template<typename DataType, typename Identifier,
  typename Stream =
  decltype(std::declval<std::istream &>() >> std::declval<DataType &>()),
  typename std::enable_if<
      !IsSharedPtr<DataType>::value &&
      std::is_convertible<Stream, std::istream &>::value,
      int>::type = 0>
std::istream &fromStream(std::istream &_in, DataType &_data)
{
  _in >> _data;
  return _in;
}

/// \brief Helper template to call stream operators only on types that support
/// them.
/// This version is called for types that are pointers to types that have
/// operator>>
/// \tparam DataType Type on which the operator will be called.
/// \tparam Identifier Unique identifier for the component class.
/// \tparam Stream Type used to check if component has operator<<
/// \param[in] _out Out stream.
/// \param[in] _data Data to be serialized.
template<typename DataType, typename Identifier,
  typename Stream =
  decltype(std::declval<std::istream &>() >> std::declval<
    typename DataType::element_type &>()),
  typename std::enable_if<
    IsSharedPtr<DataType>::value &&
    std::is_convertible<Stream, std::istream &>::value,
    int>::type = 0>
std::istream &fromStream(std::istream &_in, DataType &_data)
{
  _in >> *_data;
  return _in;
}

/// \brief Helper template to call extract operators only on types that support
/// them.
/// \tparam DataType Type on which the operator will be called.
/// \tparam Identifier Unique identifier for the component class.
/// \tparam Ignored All other template parameters are ignored.
/// This version is called for types that don't have operator>>
/// \param[in] _in In stream.
/// \param[in] _data Data resulting from deserialization.
template<typename DataType, typename Identifier, typename... Ignored>
std::istream &fromStream(std::istream &_in, DataType const &,
    Ignored const &..., ...)
{
  static bool warned{false};
  if (!warned)
  {
    ignwarn << "Trying to deserialize component with data type ["
            << typeid(DataType).name() << "], which doesn't have "
            << "`operator>>`. Component will not be deserialized." << std::endl;
    warned = true;
  }
  return _in;
}

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

    /// \brief Stream insertion operator. It exposes the component's serialized
    /// state which can be recreated by `operator>>`.
    ///
    /// \internal This function is called when using the base class, even if
    /// the component can be casted to a derived class.
    ///
    /// \param[in] _out Output stream.
    /// \param[in] _component The component to be streamed.
    /// \return The stream.
    public: friend std::ostream &operator<<(
                std::ostream &_out, const BaseComponent &_component)
    {
      _component.Serialize(_out);
      return _out;
    }

    /// \brief Stream extraction operator. It parses the component's serialized
    /// state which is created by `operator<<`.
    ///
    /// \internal This function is called when using the base class, even if
    /// the component can be casted to a derived class.
    ///
    /// \param[in] _in Input stream.
    /// \param[in] _component The component to be populated.
    /// \return The stream.
    public: friend std::istream &operator>>(
                std::istream &_in, BaseComponent &_component)
    {
      _component.Deserialize(_in);
      return _in;
    }

    /// \brief Fills a stream with a serialized version of the component.
    /// By default, it will leave the stream empty. Derived classes should
    /// override this function to support serialization.
    ///
    /// \internal This function is used by `operator<<`, which can't be
    /// overridden by derived classes.
    ///
    /// \param[in] _out Out stream.
    protected: virtual void Serialize(std::ostream &/*_out*/) const
    {
      static bool warned{false};
      if (!warned)
      {
        ignwarn << "Trying to serialize copmponent of type [" << this->TypeId()
                << "], which hasn't implemented the `Serialize` function. "
                << "Component will not be serialized."
                << std::endl;
        warned = true;
      }
    };

    /// \brief Fills a component based on a stream with a serialized data.
    /// By default, it will do nothing. Derived classes should
    /// override this function to support deserialization.
    ///
    /// \internal This function is used by `operator>>`, which can't be
    /// overridden by derived classes.
    ///
    /// \param[in] _in In stream.
    protected: virtual void Deserialize(std::istream &/*_in*/)
    {
      static bool warned{false};
      if (!warned)
      {
        ignwarn << "Trying to deserialize copmponent of type ["
                << this->TypeId() << "], which hasn't implemented the "
                << "`Deserialize` function. Component will not be deserialized."
                << std::endl;
        warned = true;
      }
    };

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
    public: ComponentTypeId TypeId() const override;

    // Documentation inherited
    public: void Serialize(std::ostream &_out) const override;

    // Documentation inherited
    public: void Deserialize(std::istream &_in) override;

    /// \brief Get the mutable component data.
    /// \return Mutable reference to the actual component information.
    public: DataType &Data();

    /// \brief Get the immutable component data.
    /// \return Immutable reference to the actual component information.
    public: const DataType &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<ComponentPrivate<DataType>> dataPtr;

    /// \brief Unique ID for this component type. This is set through the
    /// Factory registration.
    public: inline static ComponentTypeId typeId{0};

    /// \brief Unique name for this component type. This is set through the
    /// Factory registration.
    public: inline static std::string typeName;
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
    /// \brief Components with no data are always equal to another instance of
    /// the same type.
    /// \param[in] _component Component to compare to
    /// \return True.
    public: bool operator==(const Component<NoData, Identifier> &) const;

    /// \brief Components with no data are always equal to another instance of
    /// the same type.
    /// \param[in] _component Component to compare to
    /// \return False.
    public: bool operator!=(const Component<NoData, Identifier> &) const;

    /// \brief Components with no data are always serialize to an empty string.
    /// \param[in] _out Out stream.
    /// \param[in] _component Component to stream
    /// \return The same _out stream, unchanged.
    public: friend std::ostream &operator<<(std::ostream &_out,
        const Component<NoData, Identifier> &)
    {
      return _out;
    }

    /// \brief Components with no data are always serialize to an empty string.
    /// \param[in] _out In stream.
    /// \param[in] _component Component to stream
    /// \return The same _in stream, unchanged.
    public: friend std::istream &operator>>(std::istream &_in,
        Component<NoData, Identifier> &)
    {
      return _in;
    }

    // Documentation inherited
    public: ComponentTypeId TypeId() const override;

    /// \brief Unique ID for this component type. This is set through the
    /// Factory registration.
    public: inline static ComponentTypeId typeId{0};

    /// \brief Unique name for this component type. This is set through the
    /// Factory registration.
    public: inline static std::string typeName;
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
  DataType &Component<DataType, Identifier>::Data()
  {
    return this->dataPtr->data;
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
  void Component<DataType, Identifier>::Serialize(std::ostream &_out) const
  {
    toStream<DataType, Identifier>(_out, this->Data());
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  void Component<DataType, Identifier>::Deserialize(std::istream &_in)
  {
    fromStream<DataType, Identifier>(_in, this->Data());
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
  ComponentTypeId Component<NoData, Identifier>::TypeId() const
  {
    return typeId;
  }
}
}
}
}
#endif
