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
#ifndef GZ_SIM_COMPONENTS_MODEL_HH_
#define GZ_SIM_COMPONENTS_MODEL_HH_

#include <string>

#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include <gz/sim/components/Factory.hh>
#include <gz/sim/components/Component.hh>
#include <gz/sim/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace serializers
{
  class SdfModelSerializer
  {
    /// \brief Serialization for `sdf::Model`.
    /// \param[in] _out Output stream.
    /// \param[in] _time Model to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                const sdf::Model &)
    {
      // Skip serialization of model sdf
      // \todo(iche033) It was found that deserialization is
      // very expensive, which impacts initial load time of a world with many
      // entities. Currently only the server consumes ModelSdf components
      // so let's skip serialization until either the deserialization
      // performance is improved or when the GUI needs the to use Model SDF
      // components.
      // see, https://github.com/gazebosim/sdformat/issues/1478
      _out << "";
      return _out;
    }

    /// \brief Deserialization for `sdf::Model`.
    /// \param[in] _in Input stream.
    /// \param[out] _model Model to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                sdf::Model &)
    {
      // Its super expensive to create an SDFElement for some reason.
      // So seriazliation / deserialization of model sdf is skipped
      // https://github.com/gazebosim/sdformat/issues/1478
      return _in;
    }
  };
}

namespace components
{
  /// \brief A component that identifies an entity as being a model.
  using Model = Component<NoData, class ModelTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.Model", Model)

  /// \brief A component that holds the model's SDF DOM
  using ModelSdf = Component<sdf::Model,
                   class ModelTag,
                   serializers::SdfModelSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.ModelSdf", ModelSdf)
}
}
}
}

#endif
