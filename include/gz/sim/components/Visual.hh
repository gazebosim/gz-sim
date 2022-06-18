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
#ifndef GZ_SIM_COMPONENTS_VISUAL_HH_
#define GZ_SIM_COMPONENTS_VISUAL_HH_

#include <string>

#include <sdf/parser.hh>
#include <sdf/Element.hh>

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
  class SdfElementSerializer
  {
    /// \brief Serialization for `sdf::Model`.
    /// \param[in] _out Output stream.
    /// \param[in] _elem Visual plugin elem to stream
    /// \return The stream.
    public: static std::ostream &Serialize(std::ostream &_out,
                const sdf::ElementPtr &_elem)
    {
      _out << "<?xml version=\"1.0\" ?>"
           << "<sdf version='" << SDF_PROTOCOL_VERSION << "'>"
           << _elem->ToString("")
           << "</sdf>";
      return _out;
    }

    /// \brief Deserialization for `sdf::Element`.
    /// \param[in] _in Input stream.
    /// \param[out] _elem Visual plugin elem to populate
    /// \return The stream.
    public: static std::istream &Deserialize(std::istream &_in,
                sdf::ElementPtr &_elem)
    {
      std::string sdfStr(std::istreambuf_iterator<char>(_in), {});

      sdf::SDFPtr sdfParsed(new sdf::SDF());
      sdf::init(sdfParsed);
      bool result = sdf::readString(sdfStr, sdfParsed);
      if (!result)
      {
        gzerr << "Unable to deserialize sdf::ElementPtr" << std::endl;
        return _in;
      }

      _elem = sdfParsed->Root()->GetFirstElement();
      return _in;
    }
  };
}

namespace components
{
  /// \brief A component that identifies an entity as being a visual.
  using Visual = Component<NoData, class VisualTag>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.Visual", Visual)

  /// \brief A component that contains a visual plugin's SDF element.
  using VisualPlugin = Component<sdf::ElementPtr,
                                 class VisualPluginTag,
                                 serializers::SdfElementSerializer>;
  GZ_SIM_REGISTER_COMPONENT("gz_sim_components.VisualPlugin",
      VisualPlugin)
}
}
}
}

#endif
