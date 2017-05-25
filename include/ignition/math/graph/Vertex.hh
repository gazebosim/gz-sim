/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef IGNITION_MATH_GRAPH_VERTEX_HH_
#define IGNITION_MATH_GRAPH_VERTEX_HH_

// uint64_t
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <utility>

#include <ignition/math/Helpers.hh>

namespace ignition
{
namespace math
{
namespace graph
{
  /// \def VertexId.
  /// \brief The unique Id of each vertex.
  using VertexId = uint64_t;

  /// \brief Represents an invalid Id.
  static const VertexId kNullId = MAX_UI64;

  /// \brief A vertex of a graph. It stores user information and keeps
  /// an internal unique Id.
  template<typename V>
  class Vertex
  {
    /// \brief An invalid vertex.
    public: static Vertex<V> NullVertex;

    /// \brief Constructor.
    /// \param[in] _name Non-unique vertex name.
    /// \param[in] _data User information.
    /// \param[in] _id Optional unique id.
    public: Vertex(const std::string &_name,
                   const V &_data,
                   const VertexId _id = kNullId)
      : name(_name),
        data(_data),
        id(_id)
    {
    }

    /// \brief Retrieve the user information.
    /// \return Reference to the user information.
    public: const V &Data() const
    {
      return this->data;
    }

    /// \brief Get a mutable reference to the user information.
    /// \return Mutable reference to the user information.
    public: V &Data()
    {
      return this->data;
    }

    /// \brief Get the vertex Id.
    /// \return The vertex Id.
    public: VertexId Id() const
    {
      return this->id;
    }

    /// \brief Get the vertex name.
    /// \return The vertex name.
    public: std::string Name() const
    {
      return this->name;
    }

    /// \brief Whether the vertex is considered valid or not (id==kNullId).
    /// \return True when the vertex is valid or false otherwise (id==kNullId)
    public: bool Valid() const
    {
      return this->id != kNullId;
    }

    /// \brief Stream insertion operator. The output uses DOT graph
    /// description language.
    /// \param[out] _out The output stream.
    /// \param[in] _v Vertex to write to the stream.
    /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const Vertex<V> &_v)
    {
      _out << "  " << _v.Id() << " [label=\"" << _v.Name()
           << " (" << _v.Id() << ")\"];" << std::endl;
      return _out;
    }

    /// \brief Non-unique vertex name.
    private: std::string name = "";

    /// \brief User information.
    private: V data;

    /// \brief Unique vertex Id.
    private: VertexId id = kNullId;
  };

  /// \brief An invalid vertex.
  template<typename V>
  Vertex<V> Vertex<V>::NullVertex("__null__", V(), kNullId);

  /// \def VertexId_P
  /// \brief A pair of two vertex Ids.
  using VertexId_P = std::pair<VertexId, VertexId>;

  /// \def VertexRef_M
  /// \brief Map of vertices. The key is the vertex Id. The value is a
  /// reference to the vertex.
  template<typename V>
  using VertexRef_M =
    std::map<VertexId, std::reference_wrapper<const Vertex<V>>>;
}
}
}
#endif
