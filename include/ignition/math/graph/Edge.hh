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
#ifndef IGNITION_MATH_GRAPH_EDGE_HH_
#define IGNITION_MATH_GRAPH_EDGE_HH_

// uint64_t
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <set>

#include <ignition/math/config.hh>
#include "ignition/math/graph/Vertex.hh"

namespace ignition
{
namespace math
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_MATH_VERSION_NAMESPACE {
namespace graph
{
  /// \def EdgeId.
  /// \brief The unique Id for an edge.
  using EdgeId = uint64_t;

  /// \brief Used in the Graph constructors for uniform initialization.
  template<typename E>
  struct EdgeInitializer
  {
    /// \brief Constructor.
    /// \param[in] _vertices The vertices of the edge.
    /// \param[in] _data The data stored in the edge.
    /// \param[in] _weight The weight (cost) of the edge.
    EdgeInitializer(const VertexId_P &_vertices,
                    const E &_data = E(),
                    const double _weight = 1)
      : vertices(_vertices),
        data(_data),
        weight(_weight)
    {
    };

    /// \brief IDs of the vertices.
    public: VertexId_P vertices;

    /// \brief User data.
    public: E data;

    /// \brief The weight (cost) of the edge.
    public: double weight = 1;
  };

  /// \brief Generic edge class. An edge has two ends and some constraint
  /// between them. For example, a directed edge only allows traversing the
  /// edge in one direction.
  template<typename E>
  class Edge
  {
    /// \brief Constructor.
    /// \param[in] _vertices The vertices of the edge.
    /// \param[in] _data The data stored in the edge.
    /// \param[in] _weight The weight (cost) of the edge.
    /// \param[in] _id Optional unique id.
    public: explicit Edge(const VertexId_P &_vertices,
                          const E &_data,
                          const double _weight,
                          const EdgeId &_id = kNullId)
      : id(_id),
        vertices(_vertices),
        data(_data),
        weight(_weight)
    {
    }

    /// \brief Get the edge Id.
    /// \return The edge Id.
    public: EdgeId Id() const
    {
      return this->id;
    }

    /// \brief Get the two vertices contained in the edge.
    /// \return The two vertices contained in the edge.
    public: VertexId_P Vertices() const
    {
      if (!this->Valid())
        return {kNullId, kNullId};

      return this->vertices;
    }

    /// \brief Get a non-mutable reference to the user data stored in the edge
    /// \return The non-mutable reference to the user data stored in the edge.
    public: const E &Data() const
    {
      return this->data;
    }

    /// \brief Get a mutable reference to the user data stored in the edge.
    /// \return The mutable reference to the user data stored in the edge.
    public: E &Data()
    {
      return this->data;
    }

    /// \brief The cost of traversing the _from end to the other end of the
    /// edge.
    /// \return The cost.
    public: double Weight() const
    {
      return this->weight;
    }

    /// \brief Set the cost of the edge.
    /// \param[in] _newWeight The new cost.
    public: void SetWeight(const double _newWeight)
    {
      this->weight = _newWeight;
    }

    /// \brief Get the destination end that is reachable from a source end of
    /// an edge.
    ///
    /// E.g.: Let's assume that we have an undirected edge (e1) with ends
    /// (v1) and (v2): (v1)--(v2). The operation e1.From(v1) returns (v2).
    /// The operation e1.From(v2) returns (v1).
    ///
    /// E.g.: Let's assume that we have a directed edge (e2) with the tail end
    /// (v1) and the head end (v2): (v1)->(v2). The operation e2.From(v1)
    /// returns (v2). The operation e2.From(v2) returns kNullId.
    ///
    /// \param[in] _from Source vertex.
    /// \return The other vertex of the edge reachable from the "_from"
    /// vertex or kNullId otherwise.
    public: virtual VertexId From(const VertexId &_from) const = 0;

    /// \brief Get the source end that can reach the destination end of
    /// an edge.
    ///
    /// E.g.: Let's assume that we have an undirected edge (e1) with ends
    /// (v1) and (v2): (v1)--(v2). The operation e1.To(v1) returns (v2).
    /// The operation e1.To(v2) returns (v1).
    ///
    /// E.g.: Let's assume that we have a directed edge (e2) with the tail end
    /// (v1) and the head end (v2): (v1)->(v2). The operation e2.To(v1)
    /// returns kNullId. The operation e2.To(v2) returns v1.
    ///
    /// \param[in] _from Destination vertex.
    /// \return The other vertex of the edge that can reach "_to"
    /// vertex or kNullId otherwise.
    public: virtual VertexId To(const VertexId &_to) const = 0;

    /// \brief An edge is considered valid when its id is not kNullId.
    /// \return Whether the edge is valid or not.
    public: bool Valid() const
    {
      return this->id != kNullId;
    }

    /// \brief Unique edge Id.
    private: EdgeId id = kNullId;

    /// \brief The set of Ids of the two vertices.
    private: VertexId_P vertices;

    /// \brief User data.
    private: E data;

    /// \brief The weight (cost) of the edge. By default, the cost of an edge
    /// is 1.0 .
    private: double weight = 1.0;
  };

  /// \def EdgeId_S
  /// \brief A set of edge Ids.
  using EdgeId_S = std::set<EdgeId>;

  /// \def EdgeRef_M
  /// \brief A map of edges. The key is the edge Id. The value is a reference
  /// to the edge.
  template<typename EdgeType>
  using EdgeRef_M = std::map<EdgeId, std::reference_wrapper<const EdgeType>>;

  /// \brief An undirected edge represents a connection between two vertices.
  /// The connection is bidirectional, it's possible to traverse the edge
  /// in both directions.
  template<typename E>
  class UndirectedEdge : public Edge<E>
  {
    /// \brief An invalid undirected edge.
    public: static UndirectedEdge<E> NullEdge;

    /// \brief Constructor.
    /// \param[in] _vertices The vertices of the edge.
    /// \param[in] _data The data stored in the edge.
    /// \param[in] _weight The weight (cost) of the edge.
    /// \param[in] _id Optional unique id.
    public: explicit UndirectedEdge(const VertexId_P &_vertices,
                                    const E &_data,
                                    const double _weight,
                                    const EdgeId &_id = kNullId)
      : Edge<E>(_vertices, _data, _weight, _id)
    {
    }

    // Documentation inherited.
    public: VertexId From(const VertexId &_from) const override
    {
      if (!this->Valid())
        return kNullId;

      if (this->Vertices().first != _from && this->Vertices().second != _from)
        return kNullId;

      if (this->Vertices().first == _from)
        return this->Vertices().second;

      return this->Vertices().first;
    }

    // Documentation inherited.
    public: VertexId To(const VertexId &_to) const override
    {
      return this->From(_to);
    }

    /// \brief Stream insertion operator. The output uses DOT graph
    /// description language.
    /// \param[out] _out The output stream.
    /// \param[in] _e Edge to write to the stream.
    /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const UndirectedEdge<E> &_e)
    {
      auto vertices = _e.Vertices();
      _out << "  " << vertices.first << " -- " << vertices.second
           << " [label=" << _e.Weight() << "];" << std::endl;
      return _out;
    }
  };

  /// \brief An invalid undirected edge.
  template<typename E>
  UndirectedEdge<E> UndirectedEdge<E>::NullEdge(
    VertexId_P(kNullId, kNullId), E(), 1.0, kNullId);

  /// \brief A directed edge represents a connection between two vertices.
  /// The connection is unidirectional, it's only possible to traverse the edge
  /// in one direction (from the tail to the head).
  template<typename E>
  class DirectedEdge : public Edge<E>
  {
    /// \brief An invalid directed edge.
    public: static DirectedEdge<E> NullEdge;

    /// \brief Constructor.
    /// \param[in] _vertices The vertices of the edge.
    /// \param[in] _data The data stored in the edge.
    /// \param[in] _weight The weight (cost) of the edge.
    /// \param[in] _id Optional unique id.
    public: explicit DirectedEdge(const VertexId_P &_vertices,
                                  const E &_data,
                                  const double _weight,
                                  const EdgeId &_id = kNullId)
      : Edge<E>(_vertices, _data, _weight, _id)
    {
    }

    /// \brief Get the Id of the tail vertex in this edge.
    /// \return An id of the tail vertex in this edge.
    /// \sa Head()
    public: VertexId Tail() const
    {
      return this->Vertices().first;
    }

    /// \brief Get the Id of the head vertex in this edge.
    /// \return An id of the head vertex in this edge.
    /// \sa Tail()
    public: VertexId Head() const
    {
      return this->Vertices().second;
    }

    // Documentation inherited.
    public: VertexId From(const VertexId &_from) const override
    {
      if (_from != this->Tail())
        return kNullId;

      return this->Head();
    }

    // Documentation inherited.
    public: VertexId To(const VertexId &_to) const override
    {
      if (_to != this->Head())
        return kNullId;

      return this->Tail();
    }

    /// \brief Stream insertion operator. The output uses DOT graph
    /// description language.
    /// \param[out] _out The output stream.
    /// \param[in] _e Edge to write to the stream.
    /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
    public: friend std::ostream &operator<<(std::ostream &_out,
                                            const DirectedEdge<E> &_e)
    {
      _out << "  " << _e.Tail() << " -> " << _e.Head()
           << " [label=" << _e.Weight() << "];" << std::endl;
      return _out;
    }
  };

  /// \brief An invalid directed edge.
  template<typename E>
  DirectedEdge<E> DirectedEdge<E>::NullEdge(
    VertexId_P(kNullId, kNullId), E(), 1.0, kNullId);
}
}
}
}
#endif
