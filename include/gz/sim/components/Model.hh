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
                const sdf::Model &_model)
    {
      sdf::ElementPtr modelElem = _model.Element();
      if (!modelElem)
      {
        gzerr << "Unable to serialize sdf::Model" << std::endl;
        return _out;
      }

      _out << "<?xml version=\"1.0\" ?>"
           << "<sdf version='" << SDF_PROTOCOL_VERSION << "'>"
           << modelElem->ToString("")
           << "</sdf>";
      return _out;
    }

    /// \brief Deserialization for `sdf::Model`.
    /// \param[in] _in Input stream.
    /// \param[out] _model Model to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                sdf::Model &_model)
    {
      sdf::Root root;
      std::string sdf(std::istreambuf_iterator<char>(_in), {});

      sdf::Errors errors = root.LoadSdfString(sdf);
      if (!root.Model())
      {
        gzerr << "Unable to unserialize sdf::Model" << std::endl;
        return _in;
      }

      _model = *root.Model();
      return _in;
    }
  };
}

namespace components
{
  /// \brief A component that identifies an entity as being a model.
  using Model = Component<NoData, class ModelTag>;
  IGN_GAZEBO_REGISTER_COMPONENT("gz_sim_components.Model", Model)

  /// \brief A component that holds the model's SDF DOM
  using ModelSdf = Component<sdf::Model,
                   class ModelTag,
                   serializers::SdfModelSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("gz_sim_components.ModelSdf", ModelSdf)
}
}
}
}

#endif
