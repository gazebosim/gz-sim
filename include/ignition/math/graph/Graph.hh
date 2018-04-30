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
#ifndef IGNITION_MATH_GRAPH_GRAPH_HH_
#define IGNITION_MATH_GRAPH_GRAPH_HH_

#include <cassert>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <ignition/math/config.hh>
#include "ignition/math/graph/Edge.hh"
#include "ignition/math/graph/Vertex.hh"

namespace ignition
{
namespace math
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_MATH_VERSION_NAMESPACE {
namespace graph
{
  /// \brief A generic graph class.
  /// Both vertices and edges can store user information. A vertex could be
  /// created passing a custom Id if needed, otherwise it will be choosen
  /// internally. The vertices also have a name that could be reused among
  /// other vertices if needed. This class supports the use of different edge
  /// types (e.g. directed or undirected edges).
  ///
  /// <b> Example directed graph</b>
  //
  /// \code{.cpp}
  ///
  /// // Create a directed graph that is capable of storing integer data in the
  /// // vertices and double data on the edges.
  /// ignition::math::graph::DirectedGraph<int, double> graph(
  ///   // Create the vertices, with default data and vertex ids.
  ///   {
  ///     {"vertex1"}, {"vertex2"}, {"vertex3"}
  ///   },
  ///   // Create the edges, with default data and weight values.
  ///   {
  ///     // Edge from vertex 0 to vertex 1. Each number refers to a vertex id.
  ///     // Vertex ids start from zero.
  ///     {{0, 1}}, {{1, 2}}
  ///   });
  ///
  /// // You can assign data to vertices.
  /// ignition::math::graph::DirectedGraph<int, double> graph2(
  ///   // Create the vertices, with custom data and default vertex ids.
  ///   {
  ///     {"vertex1", 1}, {"vertex2", 2}, {"vertex3", 10}
  ///   },
  ///   // Create the edges, with default data and weight values.
  ///   {
  ///     // Edge from vertex 0 to vertex 1. Each number refers to a vertex id
  ///     // specified above.
  ///     {{0, 2}}, {{1, 2}}
  ///   });
  ///
  ///
  /// // It's also possible to specify vertex ids.
  /// ignition::math::graph::DirectedGraph<int, double> graph3(
  ///   // Create the vertices with custom data and vertex ids.
  ///   {
  ///     {"vertex1", 1, 2}, {"vertex2", 2, 3}, {"vertex3", 10, 4}
  ///   },
  ///   // Create the edges, with custom data and default weight values.
  ///   {
  ///     {{2, 3}, 6.3}, {{3, 4}, 4.2}
  ///   });
  ///
  /// // Finally, you can also assign weights to the edges.
  /// ignition::math::graph::DirectedGraph<int, double> graph4(
  ///   // Create the vertices with custom data and vertex ids.
  ///   {
  ///     {"vertex1", 1, 2}, {"vertex2", 2, 3}, {"vertex3", 10, 4}
  ///   },
  ///   // Create the edges, with custom data and default weight values
  ///   {
  ///     {{2, 3}, 6.3, 1.1}, {{3, 4}, 4.2, 2.3}
  ///   });
  /// \endcode
  template<typename V, typename E, typename EdgeType>
  class Graph
  {
    /// \brief Default constructor.
    public: Graph() = default;

    /// \brief Constructor.
    /// \param[in] _vertices Collection of vertices.
    /// \param[in] _edges Collection of edges.
    public: Graph(const std::vector<Vertex<V>> &_vertices,
                  const std::vector<EdgeInitializer<E>> &_edges)
    {
      // Add all vertices.
      for (auto const &v : _vertices)
      {
        if (!this->AddVertex(v.Name(), v.Data(), v.Id()).Valid())
        {
          std::cerr << "Invalid vertex with Id [" << v.Id() << "]. Ignoring."
                    << std::endl;
        }
      }

      // Add all edges.
      for (auto const &e : _edges)
      {
        if (!this->AddEdge(e.vertices, e.data, e.weight).Valid())
          std::cerr << "Ignoring edge" << std::endl;
      }
    }

    /// \brief Add a new vertex to the graph.
    /// \param[in] _name Name of the vertex. It doesn't have to be unique.
    /// \param[in] _data Data to be stored in the vertex.
    /// \param[in] _id Optional Id to be used for this vertex. This Id must
    /// be unique.
    /// \return A reference to the new vertex.
    public: Vertex<V> &AddVertex(const std::string &_name,
                                 const V &_data,
                                 const VertexId &_id = kNullId)
    {
      auto id = _id;

      // The user didn't provide an Id, we generate it.
      if (id == kNullId)
      {
        id = this->NextVertexId();

        // No space for new Ids.
        if (id == kNullId)
        {
          std::cerr << "[Graph::AddVertex()] The limit of vertices has been "
                    << "reached. Ignoring vertex." << std::endl;
          return Vertex<V>::NullVertex;
        }
      }

      // Create the vertex.
      auto ret = this->vertices.insert(
        std::make_pair(id, Vertex<V>(_name, _data, id)));

      // The Id already exists.
      if (!ret.second)
      {
        std::cerr << "[Graph::AddVertex()] Repeated vertex [" << id << "]"
                  << std::endl;
        return Vertex<V>::NullVertex;
      }

      // Link the vertex with an empty list of edges.
      this->adjList[id] = EdgeId_S();

      // Update the map of names.
      this->names.insert(std::make_pair(_name, id));

      return ret.first->second;
    }

    /// \brief The collection of all vertices in the graph.
    /// \return A map of vertices, where keys are Ids and values are
    /// references to the vertices.
    public: const VertexRef_M<V> Vertices() const
    {
      VertexRef_M<V> res;
      for (auto const &v : this->vertices)
        res.emplace(std::make_pair(v.first, std::cref(v.second)));

      return std::move(res);
    }

    /// \brief The collection of all vertices in the graph with name == _name.
    /// \return A map of vertices, where keys are Ids and values are
    /// references to the vertices.
    public: const VertexRef_M<V> Vertices(const std::string &_name) const
    {
      VertexRef_M<V> res;
      for (auto const &vertex : this->vertices)
      {
        if (vertex.second.Name() == _name)
          res.emplace(std::make_pair(vertex.first, std::cref(vertex.second)));
      }

      return std::move(res);
    }

    /// \brief Add a new edge to the graph.
    /// \param[in] _vertices The set of Ids of the two vertices.
    /// \param[in] _data User data.
    /// \return Reference to the new edge created or NullEdge if the
    /// edge was not created (e.g. incorrect vertices).
    public: EdgeType &AddEdge(const VertexId_P &_vertices,
                              const E &_data,
                              const double _weight = 1.0)
    {
      auto id = this->NextEdgeId();

      // No space for new Ids.
      if (id == kNullId)
      {
        std::cerr << "[Graph::AddEdge()] The limit of edges has been reached. "
                  << "Ignoring edge." << std::endl;
        return EdgeType::NullEdge;
      }

      EdgeType newEdge(_vertices, _data, _weight, id);
      return this->LinkEdge(std::move(newEdge));
    }

    /// \brief Links an edge to the graph. This function verifies that the
    /// edge's two vertices exist in the graph, copies the edge into the
    /// graph's internal data structure, and returns a reference to this
    /// new edge.
    /// \param[in] _edge An edge to copy into the graph.
    /// \return A reference to the new edge.
    public: EdgeType &LinkEdge(const EdgeType &_edge)
    {
      auto edgeVertices = _edge.Vertices();

      // Sanity check: Both vertices should exist.
      for (auto const &v : {edgeVertices.first, edgeVertices.second})
      {
        if (this->vertices.find(v) == this->vertices.end())
          return EdgeType::NullEdge;
      }

      // Link the new edge.
      for (auto const &v : {edgeVertices.first, edgeVertices.second})
      {
        if (v != kNullId)
        {
          auto vertexIt = this->adjList.find(v);
          assert(vertexIt != this->adjList.end());
          vertexIt->second.insert(_edge.Id());
        }
      }

      auto ret = this->edges.insert(std::make_pair(_edge.Id(), _edge));

      // Return the new edge.
      return ret.first->second;
    }

    /// \brief The collection of all edges in the graph.
    /// \return A map of edges, where keys are Ids and values are references
    /// to the edges.
    public: const EdgeRef_M<EdgeType> Edges() const
    {
      EdgeRef_M<EdgeType> res;
      for (auto const &edge : this->edges)
      {
        res.emplace(std::make_pair(edge.first, std::cref(edge.second)));
      }

      return std::move(res);
    }

    /// \brief Get all vertices that are directly connected with one edge
    /// from a given vertex. In other words, this function will return
    /// child vertices of the given vertex (all vertices from the given
    /// vertex).  E.g. j is adjacent from i (the given vertex) if there is an
    /// edge (i->j).
    ///
    /// In an undirected graph, the result of this function will match
    /// the result provided by AdjacentsTo.
    ///
    /// \param[in] _vertex The Id of the vertex from which adjacent
    /// vertices will be returned.
    /// \return A map of vertices, where keys are Ids and values are
    /// references to the vertices. This is the set of adjacent vertices.
    /// An empty map will be returned when the _vertex is not found in the
    /// graph.
    public: VertexRef_M<V> AdjacentsFrom(const VertexId &_vertex) const
    {
      VertexRef_M<V> res;

      // Make sure the vertex exists
      auto vertexIt = this->adjList.find(_vertex);
      if (vertexIt == this->adjList.end())
        return res;

      for (auto const &edgeId : vertexIt->second)
      {
        const auto &edge = this->EdgeFromId(edgeId);
        auto neighborVertexId = edge.From(_vertex);
        if (neighborVertexId != kNullId)
        {
          const auto &neighborVertex = this->VertexFromId(neighborVertexId);
          res.emplace(
            std::make_pair(neighborVertexId, std::cref(neighborVertex)));
        }
      }

      return res;
    }

    /// \brief Get all vertices that are directly connected with one edge
    /// from a given vertex. In other words, this function will return
    /// child vertices of the given vertex (all vertices from the given
    /// vertex).  E.g. j is adjacent from i (the given vertex) if there is an
    /// edge (i->j).
    ///
    /// In an undirected graph, the result of this function will match
    /// the result provided by AdjacentsTo.
    ///
    /// \param[in] _vertex The Id of the vertex from which adjacent
    /// vertices will be returned.
    /// \return A map of vertices, where keys are Ids and values are
    /// references to the vertices. This is the set of adjacent vertices.
    /// An empty map will be returned when the _vertex is not found in the
    /// graph.
    public: VertexRef_M<V> AdjacentsFrom(const Vertex<V> &_vertex) const
    {
      return this->AdjacentsFrom(_vertex.Id());
    }

    /// \brief Get all vertices that are directly connected with one edge
    /// to a given vertex. In other words, this function will return
    /// child vertices of the given vertex (all vertices from the given
    /// vertex).
    ///
    /// In an undirected graph, the result of this function will match
    /// the result provided by AdjacentsFrom.
    ///
    /// E.g. i is adjacent to j (the given vertex) if there is an
    /// edge (i->j).
    /// \param[in] _vertex The Id of the vertex to check adjacentsTo.
    /// \return A map of vertices, where keys are Ids and values are
    /// references to the vertices. An empty map is returned if the
    /// _vertex is not present in this graph, or when there are no
    /// adjacent vertices.
    public: VertexRef_M<V> AdjacentsTo(const VertexId &_vertex) const
    {
      auto incidentEdges = this->IncidentsTo(_vertex);

      VertexRef_M<V> res;
      for (auto const &incidentEdgeRef : incidentEdges)
      {
        const auto &incidentEdgeId = incidentEdgeRef.first;
        const auto &incidentEdge = this->EdgeFromId(incidentEdgeId);
        const auto &neighborVertexId = incidentEdge.To(_vertex);
        const auto &neighborVertex = this->VertexFromId(neighborVertexId);
        res.emplace(
            std::make_pair(neighborVertexId, std::cref(neighborVertex)));
      }

      return res;
    }

    /// \brief Get all vertices that are directly connected with one edge
    /// to a given vertex. In other words, this function will return
    /// child vertices of the given vertex (all vertices from the given
    /// vertex).
    ///
    /// In an undirected graph, the result of this function will match
    /// the result provided by AdjacentsFrom.
    ///
    /// E.g. i is adjacent to j (the given vertex) if there is an
    /// edge (i->j).
    /// \param[in] _vertex The vertex to check adjacentsTo.
    /// \return A map of vertices, where keys are Ids and values are
    /// references to the vertices. An empty map is returned if the
    /// _vertex is not present in this graph, or when there are no
    /// adjacent vertices.
    public: VertexRef_M<V> AdjacentsTo(const Vertex<V> &_vertex) const
    {
      return this->AdjacentsTo(_vertex.Id());
    }

    /// \brief Get the number of edges incident to a vertex.
    /// \param[in] _vertex The vertex Id.
    /// \return The number of edges incidents to a vertex.
    public: size_t InDegree(const VertexId &_vertex) const
    {
      return this->IncidentsTo(_vertex).size();
    }

    /// \brief Get the number of edges incident to a vertex.
    /// \param[in] _vertex The vertex.
    /// \return The number of edges incidents to a vertex.
    public: size_t InDegree(const Vertex<V> &_vertex) const
    {
      return this->IncidentsTo(this->VertexFromId(_vertex.Id())).size();
    }

    /// \brief Get the number of edges incident from a vertex.
    /// \param[in] _vertex The vertex Id.
    /// \return The number of edges incidents from a vertex.
    public: size_t OutDegree(const VertexId &_vertex) const
    {
      return this->IncidentsFrom(_vertex).size();
    }

    /// \brief Get the number of edges incident from a vertex.
    /// \param[in] _vertex The vertex.
    /// \return The number of edges incidents from a vertex.
    public: size_t OutDegree(const Vertex<V> &_vertex) const
    {
      return this->IncidentsFrom(this->VertexFromId(_vertex.Id())).size();
    }

    /// \brief Get the set of outgoing edges from a given vertex.
    /// \param[in] _vertex Id of the vertex.
    /// \return A map of edges, where keys are Ids and values are
    /// references to the edges. An empty map is returned when the provided
    /// vertex does not exist, or when there are no outgoing edges.
    public: const EdgeRef_M<EdgeType> IncidentsFrom(const VertexId &_vertex)
      const
    {
      EdgeRef_M<EdgeType> res;

      const auto &adjIt = this->adjList.find(_vertex);
      if (adjIt == this->adjList.end())
        return res;

      const auto &edgeIds = adjIt->second;
      for (auto const &edgeId : edgeIds)
      {
        const auto &edge = this->EdgeFromId(edgeId);
        if (edge.From(_vertex) != kNullId)
          res.emplace(std::make_pair(edge.Id(), std::cref(edge)));
      }

      return std::move(res);
    }

    /// \brief Get the set of outgoing edges from a given vertex.
    /// \param[in] _vertex The vertex.
    /// \return A map of edges, where keys are Ids and values are
    /// references to the edges. An empty map is returned when the provided
    /// vertex does not exist, or when there are no outgoing edges.
    public: const EdgeRef_M<EdgeType> IncidentsFrom(
                const Vertex<V> &_vertex) const
    {
      return this->IncidentsFrom(_vertex.Id());
    }

    /// \brief Get the set of incoming edges to a given vertex.
    /// \param[in] _vertex Id of the vertex.
    /// \return A map of edges, where keys are Ids and values are
    /// references to the edges. An empty map is returned when the provided
    /// vertex does not exist, or when there are no incoming edges.
    public: const EdgeRef_M<EdgeType> IncidentsTo(
                const VertexId &_vertex) const
    {
      EdgeRef_M<EdgeType> res;

      const auto &adjIt = this->adjList.find(_vertex);
      if (adjIt == this->adjList.end())
        return res;

      const auto &edgeIds = adjIt->second;
      for (auto const &edgeId : edgeIds)
      {
        const auto &edge = this->EdgeFromId(edgeId);
        if (edge.To(_vertex) != kNullId)
          res.emplace(std::make_pair(edge.Id(), std::cref(edge)));
      }

      return std::move(res);
    }

    /// \brief Get the set of incoming edges to a given vertex.
    /// \param[in] _vertex The vertex.
    /// \return A map of edges, where keys are Ids and values are
    /// references to the edges. An empty map is returned when the provided
    /// vertex does not exist, or when there are no incoming edges.
    public: const EdgeRef_M<EdgeType> IncidentsTo(const Vertex<V> &_vertex)
      const
    {
      return this->IncidentsTo(_vertex.Id());
    }

    /// \brief Get whether the graph is empty.
    /// \return True when there are no vertices in the graph or
    /// false otherwise.
    public: bool Empty() const
    {
      return this->vertices.empty();
    }

    /// \brief Remove an existing vertex from the graph.
    /// \param[in] _vertex Id of the vertex to be removed.
    /// \return True when the vertex was removed or false otherwise.
    public: bool RemoveVertex(const VertexId &_vertex)
    {
      auto vIt = this->vertices.find(_vertex);
      if (vIt == this->vertices.end())
        return false;

      std::string name = vIt->second.Name();

      // Remove incident edges.
      auto incidents = this->IncidentsTo(_vertex);
      for (auto edgePair : incidents)
        this->RemoveEdge(edgePair.first);

      // Remove all outgoing edges.
      incidents = this->IncidentsFrom(_vertex);
      for (auto edgePair : incidents)
        this->RemoveEdge(edgePair.first);

      // Remove the vertex (key) from the adjacency list.
      this->adjList.erase(_vertex);

      // Remove the vertex.
      this->vertices.erase(_vertex);

      // Get an iterator to all vertices sharing name.
      auto iterPair = this->names.equal_range(name);
      for (auto it = iterPair.first; it != iterPair.second; ++it)
      {
        if (it->second == _vertex)
        {
          this->names.erase(it);
          break;
        }
      }

      return true;
    }

    /// \brief Remove an existing vertex from the graph.
    /// \param[in] _vertex The vertex to be removed.
    /// \return True when the vertex was removed or false otherwise.
    public: bool RemoveVertex(Vertex<V> &_vertex)
    {
      return this->RemoveVertex(_vertex.Id());
    }

    /// \brief Remove all vertices with name == _name.
    /// \param[in] _name Name of the vertices to be removed.
    /// \return The number of vertices removed.
    public: size_t RemoveVertices(const std::string &_name)
    {
      size_t numVertices = this->names.count(_name);
      size_t result = 0;
      for (size_t i = 0; i < numVertices; ++i)
      {
        auto iter = this->names.find(_name);
        if (this->RemoveVertex(iter->second))
          ++result;
      }

      return result;
    }

    /// \brief Remove an existing edge from the graph. After the removal, it
    /// won't be possible to reach any of the vertices from the edge, unless
    /// there are other edges that connect the to vertices.
    /// \param[in] _edge Id of the edge to be removed.
    /// \return True when the edge was removed or false otherwise.
    public: bool RemoveEdge(const EdgeId &_edge)
    {
      auto edgeIt = this->edges.find(_edge);
      if (edgeIt == this->edges.end())
        return false;

      auto edgeVertices = edgeIt->second.Vertices();

      // Unlink the edge.
      for (auto const &v : {edgeVertices.first, edgeVertices.second})
      {
        if (edgeIt->second.From(v) != kNullId)
        {
          auto vertex = this->adjList.find(v);
          assert(vertex != this->adjList.end());
          vertex->second.erase(_edge);
        }
      }

      this->edges.erase(_edge);

      return true;
    }

    /// \brief Remove an existing edge from the graph. After the removal, it
    /// won't be possible to reach any of the vertices from the edge, unless
    /// there are other edges that connect the to vertices.
    /// \param[in] _edge The edge to be removed.
    /// \return True when the edge was removed or false otherwise.
    public: bool RemoveEdge(EdgeType &_edge)
    {
      return this->RemoveEdge(_edge.Id());
    }

    /// \brief Get a reference to a vertex using its Id.
    /// \param[in] _id The Id of the vertex.
    /// \return A reference to the vertex with Id = _id or NullVertex if
    /// not found.
    public: const Vertex<V> &VertexFromId(const VertexId &_id) const
    {
      auto iter = this->vertices.find(_id);
      if (iter == this->vertices.end())
        return Vertex<V>::NullVertex;

      return iter->second;
    }

    /// \brief Get a mutable reference to a vertex using its Id.
    /// \param[in] _id The Id of the vertex.
    /// \return A mutable reference to the vertex with Id = _id or NullVertex
    /// if not found.
    public: Vertex<V> &VertexFromId(const VertexId &_id)
    {
      auto iter = this->vertices.find(_id);
      if (iter == this->vertices.end())
        return Vertex<V>::NullVertex;

      return iter->second;
    }

    /// \brief Get a reference to an edge based on two vertices. A NullEdge
    /// object reference is returned if an edge with the two vertices is not
    /// found. If there are multiple edges that match the provided vertices,
    /// then first is returned.
    /// \param[in] _sourceId Source vertex Id.
    /// \param[in] _destId Destination vertex Id.
    /// \return A reference to the first edge found, or NullEdge if
    /// not found.
    public: const EdgeType &EdgeFromVertices(
                const VertexId _sourceId, const VertexId _destId) const
    {
      // Get the adjacency iterator for the source vertex.
      const typename std::map<VertexId, EdgeId_S>::const_iterator &adjIt =
        this->adjList.find(_sourceId);

      // Quit early if there is no adjacency entry
      if (adjIt == this->adjList.end())
        return EdgeType::NullEdge;

      // Loop over the edges in the source vertex's adjacency list
      for (std::set<EdgeId>::const_iterator edgIt = adjIt->second.begin();
           edgIt != adjIt->second.end(); ++edgIt)
      {
        // Get an iterator to the actual edge
        const typename std::map<EdgeId, EdgeType>::const_iterator edgeIter =
          this->edges.find(*edgIt);

        // Check if the edge has the correct source and destination.
        if (edgeIter != this->edges.end() &&
            edgeIter->second.From(_sourceId) == _destId)
        {
          assert(edgeIter->second.To(_destId) == _sourceId);
          return edgeIter->second;
        }
      }

      return EdgeType::NullEdge;
    }

    /// \brief Get a reference to an edge using its Id.
    /// \param[in] _id The Id of the edge.
    /// \return A reference to the edge with Id = _id or NullEdge if
    /// not found.
    public: const EdgeType &EdgeFromId(const EdgeId &_id) const
    {
      auto iter = this->edges.find(_id);
      if (iter == this->edges.end())
        return EdgeType::NullEdge;

      return iter->second;
    }

    /// \brief Stream insertion operator. The output uses DOT graph
    /// description language.
    /// \param[out] _out The output stream.
    /// \param[in] _g Graph to write to the stream.
    /// \ref https://en.wikipedia.org/wiki/DOT_(graph_description_language).
    public: template<typename VV, typename EE, typename EEdgeType>
    friend std::ostream &operator<<(std::ostream &_out,
                                    const Graph<VV, EE, EEdgeType> &_g);

    /// \brief Get an available Id to be assigned to a new vertex.
    /// \return The next available Id or kNullId if there aren't ids available.
    private: VertexId &NextVertexId()
    {
      while (this->vertices.find(this->nextVertexId) != this->vertices.end()
          && this->nextVertexId < MAX_UI64)
      {
        ++this->nextVertexId;
      }

      return this->nextVertexId;
    }

    /// \brief Get an available Id to be assigned to a new edge.
    /// \return The next available Id or kNullId if there aren't ids available.
    private: VertexId &NextEdgeId()
    {
      while (this->edges.find(this->nextEdgeId) != this->edges.end() &&
             this->nextEdgeId < MAX_UI64)
      {
        ++this->nextEdgeId;
      }

      return this->nextEdgeId;
    }

    /// \brief The next vertex Id to be assigned to a new vertex.
    protected: VertexId nextVertexId = 0u;

    /// \brief The next edge Id to be assigned to a new edge.
    protected: VertexId nextEdgeId = 0u;

    /// \brief The set of vertices.
    private: std::map<VertexId, Vertex<V>> vertices;

    /// \brief The set of edges.
    private: std::map<EdgeId, EdgeType> edges;

    /// \brief The adjacency list.
    /// A map where the keys are vertex Ids. For each vertex (v)
    /// with id (vId), the map value contains a set of edge Ids. Each of
    /// the edges (e) with Id (eId) represents a connected path from (v) to
    /// another vertex via (e).
    private: std::map<VertexId, EdgeId_S> adjList;

    /// \brief Association between names and vertices curently used.
    private: std::multimap<std::string, VertexId> names;
  };

  /////////////////////////////////////////////////
  /// Partial template specification for undirected edges.
  template<typename VV, typename EE>
  std::ostream &operator<<(std::ostream &_out,
                           const Graph<VV, EE, UndirectedEdge<EE>> &_g)
  {
    _out << "graph {" << std::endl;

    // All vertices with the name and Id as a "label" attribute.
    for (auto const &vertexMap : _g.Vertices())
    {
      auto vertex = vertexMap.second.get();
      _out << vertex;
    }

    // All edges.
    for (auto const &edgeMap : _g.Edges())
    {
      auto edge = edgeMap.second.get();
      _out << edge;
    }

    _out << "}" << std::endl;

    return _out;
  }

  /////////////////////////////////////////////////
  /// Partial template specification for directed edges.
  template<typename VV, typename EE>
  std::ostream &operator<<(std::ostream &_out,
                           const Graph<VV, EE, DirectedEdge<EE>> &_g)
  {
    _out << "digraph {" << std::endl;

    // All vertices with the name and Id as a "label" attribute.
    for (auto const &vertexMap : _g.Vertices())
    {
      auto vertex = vertexMap.second.get();
      _out << vertex;
    }

    // All edges.
    for (auto const &edgeMap : _g.Edges())
    {
      auto edge = edgeMap.second.get();
      _out << edge;
    }

    _out << "}" << std::endl;

    return _out;
  }

  /// \def UndirectedGraph
  /// \brief An undirected graph.
  template<typename V, typename E>
  using UndirectedGraph = Graph<V, E, UndirectedEdge<E>>;

  /// \def DirectedGraph
  /// \brief A directed graph.
  template<typename V, typename E>
  using DirectedGraph = Graph<V, E, DirectedEdge<E>>;
}
}
}
}
#endif
