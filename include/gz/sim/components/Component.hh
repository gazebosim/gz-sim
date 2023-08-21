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
#ifndef GZ_SIM_COMPONENTS_COMPONENT_HH_
#define GZ_SIM_COMPONENTS_COMPONENT_HH_

#include <cstdint>
#include <memory>
#include <string>
#include <sstream>
#include <utility>

#include <gz/common/Console.hh>

#include <gz/sim/config.hh>
#include <gz/sim/Export.hh>
#include <gz/sim/Types.hh>

namespace gz
{
namespace sim
{
// namespace gz
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace traits
{
  /// \brief Helper trait to determine if a type is shared_ptr or not
  template <typename T>
  struct IsSharedPtr : std::false_type
  {
  };

  /// \brief Helper trait to determine if a type is shared_ptr or not
  template <typename T>
  struct IsSharedPtr<std::shared_ptr<T>> : std::true_type
  {
  };

  /// \brief Type trait that determines if a operator<< is defined on `Stream`
  /// and `DataType`, i.e, it checks if the function
  /// `Stream& operator<<(Stream&, const DataType&)` exists.
  /// Example:
  /// \code
  ///    constexpr bool isDoubleOutStreamable =
  ///       IsOutStreamable<std::ostream, double>::value
  /// \endcode
  template <typename Stream, typename DataType>
  class IsOutStreamable
  {
    private: template <typename StreamArg, typename DataTypeArg>
    static auto Test(int _test)
        -> decltype(std::declval<StreamArg &>()
                    << std::declval<const DataTypeArg &>(), std::true_type());

    private: template <typename, typename>
    static auto Test(...) -> std::false_type;

    public: static constexpr bool value =  // NOLINT
                decltype(Test<Stream, DataType>(true))::value;
  };

  /// \brief Type trait that determines if a operator>> is defined on `Stream`
  /// and `DataType`, i.e, it checks if the function
  /// `Stream& operator>>(Stream&, DataType&)` exists.
  /// Example:
  /// \code
  ///    constexpr bool isDoubleInStreamable =
  ///       IsInStreamable<std::istream, double>::value
  /// \endcode
  ///
  template <typename Stream, typename DataType>
  class IsInStreamable
  {
    private: template <typename StreamArg, typename DataTypeArg>
    static auto Test(int _test)
      -> decltype(std::declval<StreamArg &>() >> std::declval<DataTypeArg &>(),
                  std::true_type());

    private: template <typename, typename>
    static auto Test(...) -> std::false_type;

    public: static constexpr bool value =  // NOLINT
                decltype(Test<Stream, DataType>(0))::value;
  };
}

namespace serializers
{
  /// \brief Default serializer template to call stream operators only on types
  /// that support them. If the stream operator is not available, a warning
  /// message is printed.
  /// \tparam DataType Type on which the operator will be called.
  template <typename DataType>
  class DefaultSerializer
  {
    /// Serialization
    public: static std::ostream &Serialize(std::ostream &_out,
                                           const DataType &_data)
    {
      // cppcheck-suppress syntaxError
      if constexpr (traits::IsSharedPtr<DataType>::value) // NOLINT
      {
        if constexpr (traits::IsOutStreamable<std::ostream,
                                   typename DataType::element_type>::value)
        {
          _out << *_data;
        }
        else
        {
          static bool warned{false};
          if (!warned)
          {
            gzwarn << "Trying to serialize component with data type ["
                    << typeid(DataType).name() << "], which doesn't have "
                    << "`operator<<`. Component will not be serialized."
                    << std::endl;
            warned = true;
          }
        }
      }
      else if constexpr (traits::IsOutStreamable<std::ostream, DataType>::value)
      {
        _out << _data;
      }
      else
      {
        static bool warned{false};
        if (!warned)
        {
          gzwarn << "Trying to serialize component with data type ["
                  << typeid(DataType).name() << "], which doesn't have "
                  << "`operator<<`. Component will not be serialized."
                  << std::endl;
          warned = true;
        }
      }
      return _out;
    }

    /// \brief Deserialization
    /// \param[in] _in In stream.
    /// \param[in] _data Data resulting from deserialization.
    public: static std::istream &Deserialize(std::istream &_in,
                                             DataType &_data)
    {
      if constexpr (traits::IsSharedPtr<DataType>::value)
      {
        if constexpr (traits::IsInStreamable<std::istream,
                                   typename DataType::element_type>::value)
        {
          _in >> *_data;
        }
        else
        {
          static bool warned{false};
          if (!warned)
          {
            gzwarn << "Trying to deserialize component with data type ["
                    << typeid(DataType).name() << "], which doesn't have "
                    << "`operator>>`. Component will not be deserialized."
                    << std::endl;
            warned = true;
          }
        }
      }
      else if constexpr (traits::IsInStreamable<std::istream, DataType>::value)
      {
        _in >> _data;
      }
      else
      {
        static bool warned{false};
        if (!warned)
        {
          gzwarn << "Trying to deserialize component with data type ["
                  << typeid(DataType).name() << "], which doesn't have "
                  << "`operator>>`. Component will not be deserialized."
                  << std::endl;
          warned = true;
        }
      }
      return _in;
    }
  };
}

namespace components
{
  /// \brief Convenient type to be used by components that don't wrap any data.
  /// I.e. they act as tags and their presence is enough to infer something
  /// about the entity.
  using NoData = std::add_lvalue_reference<void>;
}

namespace serializers
{
  /// \brief Specialization of DefaultSerializer for NoData
  template<> class DefaultSerializer<components::NoData>
  {
    public: static std::ostream &Serialize(std::ostream &_out)
    {
      _out << "-";
      return _out;
    }

    public: static std::istream &Deserialize(std::istream &_in)
    {
      return _in;
    }
  };
}

namespace components
{
  /// \brief Base class for all components.
  class BaseComponent
  {
    /// \brief Default constructor.
    public: BaseComponent() = default;

    /// \brief Default destructor.
    public: virtual ~BaseComponent() = default;

    /// \brief Fills a stream with a serialized version of the component.
    /// By default, it will leave the stream empty. Derived classes should
    /// override this function to support serialization.
    ///
    /// \param[in] _out Out stream.
    public: virtual void Serialize(std::ostream &_out) const
    {
      // This will avoid a doxygen warning
      (void)_out;
      static bool warned{false};
      if (!warned)
      {
        gzwarn << "Trying to serialize component of type [" << this->TypeId()
                << "], which hasn't implemented the `Serialize` function. "
                << "Component will not be serialized." << std::endl;
        warned = true;
      }
    };

    /// \brief Fills a component based on a stream with a serialized data.
    /// By default, it will do nothing. Derived classes should
    /// override this function to support deserialization.
    ///
    /// \param[in] _in In stream.
    public: virtual void Deserialize(std::istream &_in)
    {
      // This will avoid a doxygen warning
      (void)_in;
      static bool warned{false};
      if (!warned)
      {
        gzwarn << "Trying to deserialize component of type ["
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

    /// \brief Clone the component.
    /// \return A pointer to the component.
    public: virtual std::unique_ptr<BaseComponent> Clone() const = 0;
  };

  /// \brief A component type that wraps any data type. The intention is for
  /// this class to be used to create simple components while avoiding a lot of
  /// boilerplate code. The Identifier must be a unique type so that type
  /// aliases can be used to create new components. However the type does not
  /// need to be defined anywhere
  /// eg.
  /// \code
  ///     using Static = Component<bool, class StaticTag>;
  /// \endcode
  ///
  /// Note, however, that this scheme does not have a mechanism to stop someone
  /// accidentally defining another component that wraps a bool as such:
  /// \code
  ///     using AnotherComp = Component<bool, class StaticTag>;
  /// \endcode
  /// In this case, Static and AnotherComp are exactly the same types and would
  /// not be differentiable by the EntityComponentManager.
  ///
  /// A third template argument can be passed to Component to specify the
  /// serializer class to use. If this argument is not provided, Component will
  /// use DefaultSerializer<DataType> where DataType is the first template
  /// argument to Component.
  /// eg.
  /// \code
  ///     class BoolSerializer; // Defined elsewhere
  ///     using Static = Component<bool, class StaticTag, BoolSerializer>;
  /// \endcode
  ///
  /// \tparam DataType Type of the data being wrapped by this component.
  /// \tparam Identifier Unique identifier for the component class, to avoid
  /// collision.
  /// \tparam Serializer A class that can serialize `DataType`. Defaults to a
  /// serializer that uses stream operators `<<` and `>>` on the data if they
  /// exist.
  template <typename DataType, typename Identifier,
            typename Serializer = serializers::DefaultSerializer<DataType>>
  class Component : public BaseComponent
  {
    /// \brief Alias for DataType
    public: using Type = DataType;

    /// \brief Default constructor
    public: Component() = default;

    /// \brief Constructor
    /// \param[in] _data Data to copy
    public: explicit Component(DataType _data);

    /// \brief Destructor.
    public: ~Component() override = default;

    /// \brief Equality operator.
    /// \param[in] _component Component to compare to.
    /// \return True if equal.
    public: bool operator==(const Component &_component) const;

    /// \brief Inequality operator.
    /// \param[in] _component Component to compare to.
    /// \return True if different.
    public: bool operator!=(const Component &_component) const;

    // Documentation inherited
    public: std::unique_ptr<BaseComponent> Clone() const override;

    // Documentation inherited
    public: ComponentTypeId TypeId() const override;

    // Documentation inherited
    public: void Serialize(std::ostream &_out) const override;

    // Documentation inherited
    public: void Deserialize(std::istream &_in) override;

    /// \brief Get the mutable component data. This function will be
    /// deprecated in Gazebo 3, replaced by const DataType &Data() const.
    /// Use void SetData(const DataType &) to modify data.
    /// \return Mutable reference to the actual component information.
    /// \todo(nkoenig) Deprecate this function in version 3.
    public: DataType &Data();

    /// \brief Set the data of this component.
    /// \param[in] _data New data for this component.
    /// \param[in] _eql Equality comparison function. This function should
    /// return true if two instances of DataType are equal.
    /// \return True if the _eql function returns false.
    public: bool SetData(const DataType &_data,
                const std::function<
                  bool(const DataType &, const DataType &)> &_eql);

    /// \brief Get the immutable component data.
    /// \return Immutable reference to the actual component information.
    public: const DataType &Data() const;

    /// \brief Private data pointer.
    private: DataType data;

    /// \brief Unique ID for this component type. This is set through the
    /// Factory registration.
    public: inline static ComponentTypeId typeId{0};

    /// \brief Unique name for this component type. This is set through the
    /// Factory registration.
    public: inline static const char *typeName{nullptr};
  };

  /// \brief Specialization for components that don't wrap any data.
  /// This class to be used to create simple components that represent
  /// just a "tag", while avoiding a lot of boilerplate code. The Identifier
  /// must be a unique type so that type aliases can be used to create new
  /// components. However the type does not need to be defined anywhere eg.
  ///
  ///     using Joint = Component<NoData, class JointTag>;
  ///
  template <typename Identifier, typename Serializer>
  class Component<NoData, Identifier, Serializer> : public BaseComponent
  {
    /// \brief Components with no data are always equal to another instance of
    /// the same type.
    /// \param[in] _component Component to compare to
    /// \return True.
    public: bool operator==(const Component<NoData, Identifier,
                            Serializer> &_component) const;

    /// \brief Components with no data are always equal to another instance of
    /// the same type.
    /// \param[in] _component Component to compare to
    /// \return False.
    public: bool operator!=(const Component<NoData, Identifier,
                            Serializer> &_component) const;

    // Documentation inherited
    public: std::unique_ptr<BaseComponent> Clone() const override;

    // Documentation inherited
    public: ComponentTypeId TypeId() const override;

    // Documentation inherited
    public: void Serialize(std::ostream &_out) const override;

    // Documentation inherited
    public: void Deserialize(std::istream &_in) override;

    /// \brief Unique ID for this component type. This is set through the
    /// Factory registration.
    public: inline static ComponentTypeId typeId{0};

    /// \brief Unique name for this component type. This is set through the
    /// Factory registration.
    public: inline static const char *typeName{nullptr};
  };

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  Component<DataType, Identifier, Serializer>::Component(DataType _data)
    : data(std::move(_data))
  {
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  DataType &Component<DataType, Identifier, Serializer>::Data()
  {
    return this->data;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  bool Component<DataType, Identifier, Serializer>::SetData(
      const DataType &_data,
      const std::function<bool(const DataType &, const DataType &)> &_eql)
  {
    bool result = !_eql(_data, this->data);
    this->data = _data;
    return result;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  const DataType &Component<DataType, Identifier, Serializer>::Data() const
  {
    return this->data;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  bool Component<DataType, Identifier, Serializer>::operator==(
      const Component<DataType, Identifier, Serializer> &_component) const
  {
    return this->data == _component.Data();
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  bool Component<DataType, Identifier, Serializer>::operator!=(
      const Component<DataType, Identifier, Serializer> &_component) const
  {
    return this->data != _component.Data();
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  void Component<DataType, Identifier, Serializer>::Serialize(
      std::ostream &_out) const
  {
    Serializer::Serialize(_out, this->Data());
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  void Component<DataType, Identifier, Serializer>::Deserialize(
      std::istream &_in)
  {
    Serializer::Deserialize(_in, this->Data());
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  std::unique_ptr<BaseComponent>
  Component<DataType, Identifier, Serializer>::Clone() const
  {
    Component<DataType, Identifier, Serializer> clonedComp(this->Data());
    return std::make_unique<Component<DataType, Identifier, Serializer>>(
        clonedComp);
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier, typename Serializer>
  ComponentTypeId Component<DataType, Identifier, Serializer>::TypeId() const
  {
    return typeId;
  }

  //////////////////////////////////////////////////
  template <typename Identifier, typename Serializer>
  bool Component<NoData, Identifier, Serializer>::operator==(
      const Component<NoData, Identifier, Serializer> &) const
  {
    return true;
  }

  //////////////////////////////////////////////////
  template <typename Identifier, typename Serializer>
  bool Component<NoData, Identifier, Serializer>::operator!=(
      const Component<NoData, Identifier, Serializer> &) const
  {
    return false;
  }

  //////////////////////////////////////////////////
  template <typename Identifier, typename Serializer>
  std::unique_ptr<BaseComponent>
  Component<NoData, Identifier, Serializer>::Clone() const
  {
    return std::make_unique<Component<NoData, Identifier, Serializer>>();
  }

  //////////////////////////////////////////////////
  template <typename Identifier, typename Serializer>
  ComponentTypeId Component<NoData, Identifier, Serializer>::TypeId() const
  {
    return typeId;
  }

  //////////////////////////////////////////////////
  template <typename Identifier, typename Serializer>
  void Component<NoData, Identifier, Serializer>::Serialize(
      std::ostream &_out) const
  {
    Serializer::Serialize(_out);
  }

  //////////////////////////////////////////////////
  template <typename Identifier, typename Serializer>
  void Component<NoData, Identifier, Serializer>::Deserialize(
      std::istream &_in)
  {
    Serializer::Deserialize(_in);
  }
}
}
}
}
#endif
