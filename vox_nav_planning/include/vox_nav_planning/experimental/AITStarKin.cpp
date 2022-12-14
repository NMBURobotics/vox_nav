#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>

#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/control/planners/sst/SST.h"
#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/est/EST.h"
#include "ompl/control/planners/kpiece/KPIECE1.h"
#include "ompl/control/planners/pdst/PDST.h"
#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "ompl/control/planners/PlannerIncludes.h"

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

struct VertexProperty
{
  std::uint32_t label;
  std::string name;
};
typedef float Cost;
// specify some types
typedef boost::adjacency_list<
    boost::setS,                // edge
    boost::vecS,                // vertex
    boost::undirectedS,         // type
    VertexProperty,             // vertex property
    boost::property<boost::edge_weight_t, Cost>>     // edge property
  GraphT;
typedef boost::property_map<GraphT, boost::edge_weight_t>::type WeightMap;
typedef GraphT::vertex_descriptor vertex_descriptor;
typedef GraphT::edge_descriptor edge_descriptor;
typedef GraphT::vertex_iterator vertex_iterator;
typedef std::pair<int, int> edge;

int main(int argc, char const * argv[])
{
  /* code */
  return 0;
}
