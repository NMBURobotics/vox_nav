// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef VOX_NAV_UTILITIES__BOOST_GRAPH_UTILS_HPP_
#define VOX_NAV_UTILITIES__BOOST_GRAPH_UTILS_HPP_

#include <mutex>
#include <utility>
#include <memory>

#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>

#include <pcl/point_types.h>

namespace vox_nav_utilities
{

  // Boost graph stuff for super voxel clustering based A* search
  struct VertexProperty
  {
    std::uint32_t label;
    std::string name;
    pcl::PointXYZRGBA point;
  };
  typedef float Cost;
  typedef boost::adjacency_list<
      boost::setS,              // edge
      boost::vecS,              // vertex
      boost::undirectedS,       // type
      VertexProperty,           // vertex property
      boost::property<boost::edge_weight_t, Cost>>   // edge property
    GraphT;
  typedef boost::property_map<GraphT, boost::edge_weight_t>::type WeightMap;
  typedef GraphT::vertex_descriptor vertex_descriptor;
  typedef GraphT::edge_descriptor edge_descriptor;
  typedef GraphT::vertex_iterator vertex_iterator;
  typedef std::pair<int, int> edge;

  // euclidean distance heuristic
  template<class Graph, class CostType, class SuperVoxelClustersPtr>
  class distance_heuristic : public boost::astar_heuristic<Graph, CostType>
  {
  public:
    typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
    distance_heuristic(SuperVoxelClustersPtr sc, Vertex goal_vertex, Graph g)
    : supervoxel_clusters_(sc), goal_vertex_(goal_vertex), g_(g)
    {
    }
    CostType operator()(Vertex u)
    {
      auto u_vertex_label = g_[u].label;
      auto goal_vertex_label = g_[goal_vertex_].label;
      auto u_supervoxel_centroid = supervoxel_clusters_->at(u_vertex_label)->centroid_;
      auto goal_supervoxel_centroid = supervoxel_clusters_->at(goal_vertex_label)->centroid_;
      CostType dx = u_supervoxel_centroid.x - goal_supervoxel_centroid.x;
      CostType dy = u_supervoxel_centroid.y - goal_supervoxel_centroid.y;
      CostType dz = u_supervoxel_centroid.z - goal_supervoxel_centroid.z;
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

  private:
    SuperVoxelClustersPtr supervoxel_clusters_;
    Vertex goal_vertex_;
    Graph g_;
  };

  // exception for termination
  struct FoundGoal {};
  template<class Vertex>
  class custom_goal_visitor : public boost::default_astar_visitor
  {
  public:
    custom_goal_visitor(Vertex goal_vertex, int * num_visits)
    : goal_vertex_(goal_vertex), num_visits_(num_visits)
    {
    }
    template<class Graph>
    void examine_vertex(Vertex u, Graph & g)
    {
      ++(*num_visits_);
      if (u == goal_vertex_) {
        throw FoundGoal();
      }
    }

  private:
    Vertex goal_vertex_;
    int * num_visits_;
  };

  // exception for termination
  template<class Vertex>
  class custom_goal_visitor_dijkstra : public boost::default_dijkstra_visitor
  {
  public:
    custom_goal_visitor_dijkstra(Vertex goal_vertex, int * num_visits)
    : goal_vertex_(goal_vertex), num_visits_(num_visits)
    {
    }
    template<class Graph>
    void examine_vertex(Vertex u, Graph & g)
    {
      ++(*num_visits_);
      if (u == goal_vertex_) {
        throw FoundGoal();
      }
    }

  private:
    Vertex goal_vertex_;
    int * num_visits_;
  };

}  // namespace vox_nav_utilities

#endif  // VOX_NAV_UTILITIES__BOOST_GRAPH_UTILS_HPP_
