// Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
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

#ifndef VOX_NAV_PLANNING__RRT__InformedSGCP_HPP_
#define VOX_NAV_PLANNING__RRT__InformedSGCP_HPP_

#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include "ompl/control/DirectedControlSampler.h"
#include "ompl/control/SimpleDirectedControlSampler.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/SimpleSetup.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/LPAstarOnGraph.h"
#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/util/GeometricEquations.h"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "vox_nav_utilities/elevation_state_space.hpp"

#include <thread>
#include <mutex>
#include <atomic>
#include <limits>
#include <cstdint>

namespace ompl
{
  namespace control
  {
    /**
       @anchor cInformedSGCP
       @par Short description
       \ref Informed Simultaneous Geometric and Control Planner (InformedSGCP).
       The implementation is based on boost graph library.
       An accompanying paper explaining novelities of this planner will be published soon.
       @par External documentation
       TBD
    */

    struct Parameters
    {
      /** \brief All configurable parameters of InformedSGCP. */

      /** \brief The number of threads to be used in parallel for geometric and control. */
      int num_threads_{12};

      /** \brief The number of samples to be added to graph in each iteration. */
      int batch_size_{100};

      /** \brief The a single vertex, do not construct more edges (neighbours) more than max_neighbors_. */
      int max_neighbors_{15};

      /** \brief Adding almost identical samples does not help much, so we regulate this by min_dist_between_vertices_. */
      double min_dist_between_vertices_{0.1};

      /** \brief The edges connecting samples in geometric and control graphs cannot be longer than this */
      double max_dist_between_vertices_{0.0}; // works ok for elevation

      /** \brief If available, use valid sampler. */
      bool use_valid_sampler_{false};

      /** \brief For directed control, set a number of samples to iterate though, to get a more accurate sampleTo behviour. It comes as costy!. */
      int k_number_of_controls_{2};

      /** \brief For adaptive heuristic, there is two options, dijkstra and astar.
       *  Default is dijkstra as it computes shortest path from each vertex to specified one */
      bool use_astar_hueristic_{false};

      /** \brief Frequently push goal to graph. It is used in control graph */
      double goal_bias_{0.1};

      double rewire_factor_{1.0};

      /** \brief Whether to use nearest neighbor or radius as connection strategy. */
      bool use_k_nearest_{true};

      bool solve_control_graph_{true};
    };

    class InformedSGCP : public base::Planner
    {
    public:
      /** \brief Constructor */
      InformedSGCP(const SpaceInformationPtr & si);

      ~InformedSGCP() override;

      void setup() override;

      /** \brief Continue solving for some amount of time. Return true if solution was found. */
      base::PlannerStatus solve(const base::PlannerTerminationCondition & ptc) override;

      void getPlannerData(base::PlannerData & data) const override;

      /** \brief Clear datastructures. Call this function if the
          input data to the planner has changed and you do not
          want to continue planning */
      void clear() override;

      /** \brief Free the memory allocated by this planner. That is mostly in nearest neihbours. */
      void freeMemory();

      /** \brief Properties of boost graph vertex, both geometriuc and control graphes share this vertex property.
       *  Some of the elements are not used in geometric graph (e.g., control, control_duration). */
      struct VertexProperty
      {
        ompl::base::State * state{nullptr};
        ompl::control::Control * control{nullptr};
        unsigned int control_duration{0};
        std::size_t id{0};
        double g{1.0e+3};
        bool blacklisted{false};
      };

      /** \brief Compute distance between Vertexes (actually distance between contained states) */
      double distanceFunction(const VertexProperty * a, const VertexProperty * b) const
      {
        return si_->distance(a->state, b->state);
      }

      /** \brief Compute distance between states */
      double distanceFunction(const base::State * a, const base::State * b) const
      {
        if (si_->getStateSpace()->getType() == base::STATE_SPACE_REAL_VECTOR) {
          // Use only first three elemnts of state space
          auto a_ = a->as<base::RealVectorStateSpace::StateType>();
          auto b_ = b->as<base::RealVectorStateSpace::StateType>();
          return sqrt(
            pow(a_->values[0] - b_->values[0], 2) +
            pow(a_->values[1] - b_->values[1], 2) +
            pow(a_->values[2] - b_->values[2], 2));
        } else {
          return si_->distance(a, b);
        }
      }

      /** \brief Given its vertex_descriptor (id),
       * return a const pointer to VertexProperty in geometric graph g_geometric_  */
      const VertexProperty * getVertex(std::size_t id, int thread_id)
      {
        return &g_geometrics_[thread_id][id];
      }

      /** \brief Given its vertex_descriptor (id),
       * return a mutable pointer to VertexProperty in geometric graph g_  */
      VertexProperty * getVertexMutable(std::size_t id, int thread_id)
      {
        return &g_geometrics_[thread_id][id];
      }

      /** \brief Given its vertex_descriptor (id),
       * return a const pointer to VertexProperty in control graph g_forward_control_  */
      const VertexProperty * getVertexControls(std::size_t id, int thread_id)
      {
        return &g_controls_[thread_id][id];
      }

      void setNumThreads(int num_threads)
      {
        params_.num_threads_ = num_threads;
      }

      int getNumThreads() const
      {
        return params_.num_threads_;
      }

      void setBatchSize(int batch_size)
      {
        params_.batch_size_ = batch_size;
      }

      int getBatchSize() const
      {
        return params_.batch_size_;
      }

      void setMaxNeighbors(int max_neighbors)
      {
        params_.max_neighbors_ = max_neighbors;
      }

      int getMaxNeighbors() const
      {
        return params_.max_neighbors_;
      }

      void setMinDistBetweenVertices(double min_dist_between_vertices)
      {
        params_.min_dist_between_vertices_ = min_dist_between_vertices;
      }

      double getMinDistBetweenVertices() const
      {
        return params_.min_dist_between_vertices_;
      }

      void setMaxDistBetweenVertices(double max_dist_between_vertices)
      {
        params_.max_dist_between_vertices_ = max_dist_between_vertices;
      }

      double getMaxDistBetweenVertices() const
      {
        return params_.max_dist_between_vertices_;
      }

      void setUseValidSampler(bool use_valid_sampler)
      {
        params_.use_valid_sampler_ = use_valid_sampler;
      }

      bool getUseValidSampler() const
      {
        return params_.use_valid_sampler_;
      }

      void setKNumberControls(int k_number_of_controls)
      {
        params_.k_number_of_controls_ = k_number_of_controls;
      }

      int getKNumberControls() const
      {
        return params_.k_number_of_controls_;
      }

      void setGoalBias(double goal_bias)
      {
        params_.goal_bias_ = goal_bias;
      }

      double getGoalBias() const
      {
        return params_.goal_bias_;
      }

      void setUseKNearest(bool use_k_nearest)
      {
        params_.use_k_nearest_ = use_k_nearest;
      }

      bool getUseKNearest() const
      {
        return params_.use_k_nearest_;
      }

      void setSolveControlGraph(bool solve_control_graph)
      {
        params_.solve_control_graph_ = solve_control_graph;
      }

      bool getSolveControlGraph() const
      {
        return params_.solve_control_graph_;
      }

    private:
      /** \brief All configurable parames live here. */
      Parameters params_;

      /** \brief The radius to construct edges in construction of RGG, this is meant to be used in geometric graph, determines max edge length. */
      double radius_{std::numeric_limits<double>::infinity()};

      /** \brief A constant for the computation of the number of neighbors when using a k-nearest model. */
      std::size_t k_rgg_{std::numeric_limits<std::size_t>::max()};

      /** \brief Auto calculate neighbours to connect. */
      std::size_t numNeighbors_{std::numeric_limits<std::size_t>::max()};

      int currentBestSolutionStatus_{ompl::base::PlannerStatus::UNKNOWN};

      /** \brief Informed sampling strategy */
      std::shared_ptr<base::RejectionInfSampler> rejection_informed_sampler_{nullptr};

      /** \brief Valid state sampler */
      base::ValidStateSamplerPtr valid_state_sampler_{nullptr};

      /** \brief Control space information */
      const SpaceInformation * siC_{nullptr};

      /** \brief The optimization objective. */
      base::OptimizationObjectivePtr opt_{nullptr};

      /** \brief Current cost of best path. The informed sampling strategy needs it. */
      ompl::base::Cost bestGeometricCost_{std::numeric_limits<double>::infinity()};

      /** \brief Current cost of best path. The informed sampling strategy needs it. */
      ompl::base::Cost bestControlCost_{std::numeric_limits<double>::infinity()};

      int best_control_path_index_{0};

      /** \brief The best path found so far. */
      std::shared_ptr<ompl::control::PathControl> bestGeometricPath_{nullptr};

      /** \brief The best path found so far. */
      std::shared_ptr<ompl::control::PathControl> bestControlPath_{nullptr};

      /** \brief Directed control sampler to expand control graph */
      DirectedControlSamplerPtr directedControlSampler_;

      /** \brief Control sampler */
      ControlSamplerPtr controlSampler_;

      /** \brief The random number generator */
      RNG rng_;

      /** \brief The NN datastructure for geometric graph */
      std::vector<std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>>> geometrics_nn_;

      /** \brief The NN datastructure for control graph */
      std::vector<std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>>> controls_nn_;

      /** \brief The typedef of Edge cost as double,
       * note that ompl::base::Cost wont work as some operators are not provided (e.g. +) */
      typedef double GraphEdgeCost;

      /** \brief The typedef of Graph, note that we use setS for edge container, as we want to avoid duplicate edges */
      typedef boost::adjacency_list<
          boost::setS,          // edge
          boost::vecS,          // vertex
          boost::undirectedS,   // type
          VertexProperty,       // vertex property
          boost::property<boost::edge_weight_t, GraphEdgeCost>> // edge property
        GraphT;

      /** \brief The typedef of Graph, note that we use setS for edge container, as we want to avoid duplicate edges */
      typedef boost::property_map<GraphT, boost::edge_weight_t>::type WeightMap;

      /** \brief The typedef of vertex_descriptor, note that we keep a copy of vertex_descriptor in VertexProperty as "id" */
      typedef GraphT::vertex_descriptor vertex_descriptor;

      /** \brief The typedef of edge_descriptor. */
      typedef GraphT::edge_descriptor edge_descriptor;

      /** \brief The generic eucledean distance heuristic, this is used for all graphs when we perform A* on them.
       * Make it a friend of InformedSGCP so it can access private members of InformedSGCP.
      */
      friend class GenericDistanceHeuristic;
      template<class Graph, class VertexProperty, class CostType>
      class GenericDistanceHeuristic : public boost::astar_heuristic<Graph, CostType>
      {
      public:
        typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
        GenericDistanceHeuristic(
          InformedSGCP * alg, VertexProperty * goal,
          bool control = false,
          int thread_id = 0)
        : alg_(alg),
          goal_(goal),
          control_(control),
          thread_id_(thread_id)
        {
        }
        double operator()(vertex_descriptor i)
        {
          if (control_) {
            return alg_->opt_->motionCost(
              alg_->getVertexControls(i, thread_id_)->state, goal_->state).value();
          } else {
            return alg_->opt_->motionCost(
              alg_->getVertex(i, thread_id_)->state, goal_->state).value();
          }
        }

      private:
        InformedSGCP * alg_{nullptr};
        VertexProperty * goal_{nullptr};
        bool control_{false};
        int thread_id_{0};
      };  // GenericDistanceHeuristic

      /** \brief The precomputed cost heuristic, this is used for geometric graph when we perform A* with collision checks.
       * Make it a friend of InformedSGCP so it can access private members of InformedSGCP.
      */
      friend class PrecomputedCostHeuristic;
      template<class Graph, class CostType>
      class PrecomputedCostHeuristic : public boost::astar_heuristic<Graph, CostType>
      {
      public:
        typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
        PrecomputedCostHeuristic(InformedSGCP * alg, int thread_id = 0)
        : alg_(alg),
          thread_id_(thread_id)
        {
        }
        double operator()(vertex_descriptor i)
        {
          double cost{std::numeric_limits<double>::infinity()};
          // verify that the state is valid
          if (alg_->si_->isValid(alg_->getVertex(i, thread_id_)->state)) {
            cost = alg_->getVertex(i, thread_id_)->g;
          } else {
            alg_->getVertexMutable(i, thread_id_)->blacklisted = true;
          }
          return cost;
        }

      private:
        InformedSGCP * alg_;
        int thread_id_{0};
      };  // PrecomputedCostHeuristic

      /** \brief Exception thrown when goal vertex is found */
      struct FoundVertex {};

      /** \brief The visitor class for A* search,
       *  this is used for all graphs when we perform A* on them.
      */
      template<class Vertex>
      class SimpleVertexVisitor : public boost::default_astar_visitor
      {
      public:
        SimpleVertexVisitor(Vertex goal_vertex, int * num_visits)
        : goal_vertex_(goal_vertex),
          num_visits_(num_visits)
        {
        }
        template<class Graph>
        void examine_vertex(Vertex u, Graph & g)
        {
          // check whether examined vertex was goal, if yes throw
          ++(*num_visits_);
          if (u == goal_vertex_) {
            throw FoundVertex();
          }
        }

      private:
        Vertex goal_vertex_;
        int * num_visits_;
      };  // SimpleVertexVisitor

      /** \brief Keep a global copy of start and goal vertex properties*/
      VertexProperty * geometric_start_vertex_{nullptr};
      VertexProperty * geometric_goal_vertex_{nullptr};

      std::vector<VertexProperty *> control_start_vertices_;
      std::vector<VertexProperty *> control_goal_vertices_;

      /** \brief The geometric and control graphs */
      std::vector<GraphT> g_geometrics_;

      /** \brief */
      std::vector<GraphT> g_controls_;

      /** \brief The publishers for the geometric and control graph/path visulization*/
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rgg_graph_pub_;

      /** \brief The publishers for the geometric and control graph/path visulization*/
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr geometric_path_pub_;

      /** \brief The publishers for the geometric and control graph/path visulization*/
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr control_graph_pub_;

      /** \brief The publishers for the geometric and control graph/path visulization*/
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr control_path_pub_;

      /** \brief The node*/
      rclcpp::Node::SharedPtr node_;

      /** \brief generate a requested amound of states with preffered state sampler*/
      void generateBatchofSamples(
        int batch_size,
        bool use_valid_sampler,
        std::vector<ompl::base::State *> & samples);

      /** \brief Keep expanding geometric graph with generated samples.
       * TODO(@atas), add more description here*/
      void expandGeometricGraph(
        const std::vector<ompl::base::State *> & samples,
        const base::PlannerTerminationCondition & ptc,
        GraphT & geometric_graph,
        std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & geometric_nn,
        WeightMap & geometric_weightmap);

      /** \brief In each iteration, make sure that goal vertex is connected to its nn.
       * TODO(@atas), add more description here*/
      void ensureGoalVertexConnectivity(
        VertexProperty * target_vertex_property,
        GraphT & geometric_graph,
        std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & geometric_nn,
        WeightMap & geometric_weightmap);

      /** \brief Keep expanding control graph with generated samples.
       * Note that only non-violating states will be added, the rest are discaded
       * TODO(@atas), add more description here*/
      void expandControlGraph(
        const std::vector<ompl::base::State *> & samples,
        const ompl::base::State * target_vertex_state,
        const vertex_descriptor & target_vertex_descriptor,
        const base::PlannerTerminationCondition & ptc,
        const bool & intial_plan_available,
        const std::vector<VertexProperty *> & vertex_prop_plan,
        GraphT & control_graph,
        std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & control_nn,
        WeightMap & control_weightmap,
        int & status);

      void ensureGoalVertexConnectivity(
        const ompl::base::State * target_vertex_state,
        const vertex_descriptor & target_vertex_descriptor,
        GraphT & control_graph,
        std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & control_nn,
        WeightMap & control_weightmap,
        int & status);

      template<class Heuristic>
      std::list<vertex_descriptor> computeShortestPath(
        GraphT & g,
        WeightMap & weightmap,
        const Heuristic & heuristic,
        const vertex_descriptor & start_vertex,
        const vertex_descriptor & goal_vertex,
        const bool & precompute_heuristic = false,
        const bool & use_full_collision_check = false)
      {
        std::list<vertex_descriptor> shortest_path;
        std::vector<vertex_descriptor> p(boost::num_vertices(g));
        std::vector<GraphEdgeCost> d(boost::num_vertices(g));
        int num_visited_nodes{0};
        auto visitor = SimpleVertexVisitor<vertex_descriptor>(goal_vertex, &num_visited_nodes);

        // Now we can run A* forwards from start to goal and check for collisions
        try {
          if (precompute_heuristic) {
            if (params_.use_astar_hueristic_) {
              boost::astar_search_tree(
                g, start_vertex, heuristic,
                boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(visitor));
            } else {
              boost::dijkstra_shortest_paths(
                g, start_vertex,
                boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(visitor));
            }
          } else {
            boost::astar_search_tree(
              g, start_vertex, heuristic,
              boost::predecessor_map(&p[0]).distance_map(&d[0]).visitor(visitor));
          }

          // No path found
          return shortest_path;

        } catch (FoundVertex found_goal) {

          // Catch the exception
          // Found a Heuristic from start to the goal (no collision checks),
          // We now have H function
          for (auto vd : boost::make_iterator_range(vertices(g))) {
            g[vd].g = d[vd];
          }

          // Found a collision free path to the goal, catch the exception
          for (vertex_descriptor v = goal_vertex;; v = p[v]) {
            if (use_full_collision_check) {
              if (g[v].blacklisted) {
                //   Found a blacklisted vertex most likely due to collision, Marking in/out edges as invalid for this vertex
                for (auto ed : boost::make_iterator_range(boost::out_edges(v, g))) {
                  weightmap[ed] = opt_->infiniteCost().value();
                }
              }
            }
            shortest_path.push_front(v);
            if (p[v] == v) {break;}
          }
          return shortest_path;
        }
      }

      /** \brief original AIT* function */
      std::size_t computeNumberOfSamplesInInformedSet() const;

      /** \brief original AIT* function */
      double  computeConnectionRadius(std::size_t numSamples) const;

      /** \brief original AIT* function */
      std::size_t computeNumberOfNeighbors(std::size_t numSamples) const;

      /** \brief compute path cost */
      ompl::base::Cost computePathCost(
        std::shared_ptr<ompl::control::PathControl> & path) const;

      void populateOmplPathfromVertexPath(
        const std::list<vertex_descriptor> & vertex_path,
        GraphT & g,
        WeightMap & weightmap,
        std::shared_ptr<ompl::control::PathControl> & path,
        std::vector<VertexProperty *> & vertexprop_path,
        const bool control = false
      ) const
      {

        path = std::make_shared<PathControl>(si_);
        int index{0};
        vertex_descriptor prev_vertex = vertex_path.front();


        for (auto && i : vertex_path) {

          // Use this opportunity to mark the edges as invalid if they were blacklisted
          if (g[i].blacklisted) {
            for (auto ed : boost::make_iterator_range(boost::out_edges(i, g))) {
              weightmap[ed] = opt_->infiniteCost().value();
            }
          }

          if (index > 0) {
            bool is_edge_valid = si_->checkMotion(g[prev_vertex].state, g[i].state);
            if (!is_edge_valid) {
              // Mark the edge as invalid
              auto ed = boost::edge(prev_vertex, i, g);
              weightmap[ed.first] = opt_->infiniteCost().value();
            }
            prev_vertex = i;
          }

          if (g[i].control == nullptr && control) {
            // This most likely a start or goal vertex
            // The control has not been allocated for this vertex
            // Allocate a control and set it to zero
            g[i].control = siC_->allocControl();
          }
          if (index == 0) {
            path->append(g[i].state);
          } else {
            if (control) {
              path->append(
                g[i].state, g[i].control,
                g[i].control_duration * siC_->getPropagationStepSize());
            } else {
              path->append(g[i].state);
            }
          }

          vertexprop_path[index] = new VertexProperty(g[i]);

          index++;
        }
      }

      std::vector<const ompl::base::State *> getConstStates(
        const std::shared_ptr<ompl::control::PathControl> & path)
      {
        std::vector<const ompl::base::State *> states;
        for (std::size_t i = 0; i < path->getStateCount(); ++i) {
          states.push_back(path->getState(i));
        }
        return states;
      }

      std::vector<ompl::base::State *> getStates(
        const std::shared_ptr<ompl::control::PathControl> & path)
      {
        std::vector<ompl::base::State *> states;
        for (std::size_t i = 0; i < path->getStateCount() - 1; ++i) {
          states.push_back(path->getState(i));
        }
        return states;
      }

      void clearControlGraphs(
        std::vector<WeightMap> & weightmap_controls,
        std::vector<std::pair<vertex_descriptor, vertex_descriptor>> & control_start_goal_descriptors
      )
      {
        // Reset control graphs anyways
        control_start_goal_descriptors.clear();

        for (int i = 0; i < params_.num_threads_; i++) {
          g_controls_[i].clear();
          g_controls_[i] = GraphT();
          // free memory for all nns in control threads
          controls_nn_[i]->clear();
          // Add the start and goal vertex to the control graph
          controls_nn_[i]->add(control_start_vertices_[i]);

          vertex_descriptor control_g_root = boost::add_vertex(g_controls_[i]);
          g_controls_[i][control_g_root] = *control_start_vertices_[i];
          g_controls_[i][control_g_root].id = control_g_root;
          vertex_descriptor control_g_target = boost::add_vertex(g_controls_[i]);
          g_controls_[i][control_g_target] = *control_goal_vertices_[i];
          g_controls_[i][control_g_target].id = control_g_target;

          control_start_goal_descriptors.push_back(
            std::make_pair(
              g_controls_[i][control_g_root].id,
              g_controls_[i][control_g_target].id));
        }
        // Reset the weightmap
        weightmap_controls.clear();
        for (auto & graph : g_controls_) {
          weightmap_controls.push_back(get(boost::edge_weight, graph));
        }
      }

      /** \brief static method to visulize a graph in RVIZ*/
      static void visualizeRGG(
        const GraphT & g,
        const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
        const std::string & ns,
        const std_msgs::msg::ColorRGBA & color,
        const vertex_descriptor & start_vertex,
        const vertex_descriptor & goal_vertex,
        const int & state_space_type);

      /** \brief static method to visulize a path in RVIZ*/
      static void visualizePath(
        const GraphT & g,
        const std::list<vertex_descriptor> & path,
        const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
        const std::string & ns,
        const std_msgs::msg::ColorRGBA & color,
        const int & state_space_type
      );

      static void visualizePath(
        const std::shared_ptr<PathControl> & path,
        const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr & publisher,
        const std::string & ns,
        const std_msgs::msg::ColorRGBA & color,
        const int & state_space_type
      );

      /** \brief get std_msgs::msg::ColorRGBA given the color name with a std::string*/
      static std_msgs::msg::ColorRGBA getColor(std::string & color);

    };      // class InformedSGCP
  }   // namespace control
}   // namespace ompl


#endif  // VOX_NAV_PLANNING__RRT__InformedSGCP_HPP_
