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
#ifndef IGNITION_GAZEBO_COMPONENTS_SIMPLEWRAPPER_HH_
#define IGNITION_GAZEBO_COMPONENTS_SIMPLEWRAPPER_HH_

#include <memory>
#include <utility>

#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Export.hh>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  // Forward declarations.
  template<typename DataType> class SimpleWrapperPrivate;

  /// \brief A component type that wraps any data type. The intention is for
  /// this class to be used to create simple components while avoiding a lot of
  /// boilerplate code. The Identifier must be a unique type so that type
  /// aliases can be used to create new components. However the type does not
  /// need to be defined anywhere
  /// eg.
  ///     using Static = SimpleWrapper<bool, class StaticTag>;
  ///
  /// Note, however, that this scheme does not have a mechanism to stop someone
  /// accidentally defining another component that wraps a bool as such:
  ///     using AnotherComp = SimpleWrapper<bool, class StaticTag>;
  /// In this case, Static and AnotherComp are exactly the same types and would
  /// not be differentiable by the EntityComponentManager.
  template <typename DataType, typename Identifier>
  class SimpleWrapper
  {
    /// \brief Constructor
    /// \param[in] _simpleWrapper SimpleWrapper to copy
    public: explicit SimpleWrapper(const DataType &_data);

    /// \brief Copy Constructor
    /// \param[in] _simpleWrapper SimpleWrapper component to copy.
    public: SimpleWrapper(const SimpleWrapper &_simpleWrapper);

    /// \brief Move Constructor
    /// \param[in] _simpleWrapper SimpleWrapper component to move.
    public: SimpleWrapper(SimpleWrapper &&_simpleWrapper) noexcept = default;

    /// \brief Destructor.
    public: virtual ~SimpleWrapper() = default;

    /// \brief Move assignment operator.
    /// \param[in] _simpleWrapper SimpleWrapper component to move.
    /// \return Reference to this.
    public: SimpleWrapper &operator=(
                SimpleWrapper &&_simpleWrapper) noexcept = default;

    /// \brief Copy assignment operator.
    /// \param[in] _simpleWrapper SimpleWrapper component to copy.
    /// \return Reference to this.
    public: SimpleWrapper &operator=(const SimpleWrapper &_simpleWrapper);

    /// \brief Equality operator.
    /// \param[in] _simpleWrapper SimpleWrapper to compare to.
    /// \return True if equal.
    public: bool operator==(const SimpleWrapper &_simpleWrapper) const;

    /// \brief Inequality operator.
    /// \param[in] _simpleWrapper SimpleWrapper to compare to.
    /// \return True if different.
    public: bool operator!=(const SimpleWrapper &_simpleWrapper) const;

    /// \brief Get the simpleWrapper data.
    /// \return The actual simpleWrapper information.
    public: const DataType &Data() const;

    /// \brief Private data pointer.
    private: std::unique_ptr<SimpleWrapperPrivate<DataType>> dataPtr;
  };

  template <typename DataType>
  class SimpleWrapperPrivate
  {
    /// \brief Constructor.
    /// \param[in] _simpleWrapper SimpleWrapper data.
    public: explicit SimpleWrapperPrivate(DataType _data)
            : data(std::move(_data))
    {
    }

    /// \brief The data being wrapped.
    public: DataType data;
  };

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  SimpleWrapper<DataType, Identifier>::SimpleWrapper(const DataType &_data)
    : dataPtr(std::make_unique<SimpleWrapperPrivate<DataType>>(_data))
  {
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  SimpleWrapper<DataType, Identifier>::SimpleWrapper(
      const SimpleWrapper<DataType, Identifier> &_simpleWrapper)
      : dataPtr(std::make_unique<SimpleWrapperPrivate<DataType>>(
            _simpleWrapper.Data()))
  {
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  const DataType &SimpleWrapper<DataType, Identifier>::Data() const
  {
    return this->dataPtr->data;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  SimpleWrapper<DataType, Identifier> &SimpleWrapper<DataType, Identifier>::
  operator=(const SimpleWrapper<DataType, Identifier> &_simpleWrapper)
  {
    this->dataPtr->data = _simpleWrapper.Data();
    return *this;
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  bool SimpleWrapper<DataType, Identifier>::
  operator==(const SimpleWrapper<DataType, Identifier> &_simpleWrapper) const
  {
    return this->dataPtr->data == _simpleWrapper.Data();
  }

  //////////////////////////////////////////////////
  template <typename DataType, typename Identifier>
  bool SimpleWrapper<DataType, Identifier>::
  operator!=(const SimpleWrapper<DataType, Identifier> &_simpleWrapper) const
  {
    return this->dataPtr->data != _simpleWrapper.Data();
  }
}
}
}
}
#endif
