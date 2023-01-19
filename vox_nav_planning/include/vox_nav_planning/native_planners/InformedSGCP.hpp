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
        std::size_t parent_id{0};
        double g{1.0e+3};
        bool blacklisted{false};
        bool is_root{false};
      };

      /** \brief Compute distance between Vertexes (actually distance between contained states) */
      double distanceFunction(const VertexProperty * a, const VertexProperty * b) const
      {
        return si_->distance(a->state, b->state);
      }

      /** \brief Compute distance between states */
      double distanceFunction(const base::State * a, const base::State * b) const
      {
        return si_->distance(a, b);
      }

      /** \brief Given its vertex_descriptor (id),
       * return a const pointer to VertexProperty in geometric graph g_geometric_  */
      const VertexProperty * getVertex(std::size_t id, int thread_id)
      {
        return &graphGeometricThreads_[thread_id][id];
      }

      /** \brief Given its vertex_descriptor (id),
       * return a mutable pointer to VertexProperty in geometric graph g_  */
      VertexProperty * getVertexMutable(std::size_t id, int thread_id)
      {
        return &graphGeometricThreads_[thread_id][id];
      }

      /** \brief Given its vertex_descriptor (id),
       * return a const pointer to VertexProperty in control graph g_forward_control_  */
      const VertexProperty * getVertexControls(std::size_t id, int thread_id)
      {
        return &graphControlThreads_[thread_id][id];
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
      std::size_t kRGG_{std::numeric_limits<std::size_t>::max()};

      /** \brief Auto calculate neighbours to connect. */
      std::size_t numNeighbors_{std::numeric_limits<std::size_t>::max()};

      /** \brief Keep status of current status*/
      int currentBestSolutionStatus_{ompl::base::PlannerStatus::UNKNOWN};

      /** \brief Informed sampling strategy */
      std::shared_ptr<base::RejectionInfSampler> rejectionInformedSampler_{nullptr};

      /** \brief Valid state sampler */
      base::ValidStateSamplerPtr validStateSampler_{nullptr};

      /** \brief Control space information */
      const SpaceInformation * siC_{nullptr};

      /** \brief The optimization objective. */
      base::OptimizationObjectivePtr opt_{nullptr};

      /** \brief Current cost of best path. The informed sampling strategy needs it. */
      ompl::base::Cost bestGeometricCost_{std::numeric_limits<double>::infinity()};

      /** \brief Current cost of best path. The informed sampling strategy needs it. */
      ompl::base::Cost bestControlCost_{std::numeric_limits<double>::infinity()};

      /** \brief keep the index of best control path, the index comes from thread id*/
      int bestControlPathIndex_{0};

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
      std::vector<std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>>> nnGeometricThreads_;

      /** \brief The NN datastructure for control graph */
      std::vector<std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>>> nnControlsThreads_;

      std::mutex nnMutex_;

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
          threadId_(thread_id)
        {
        }
        double operator()(vertex_descriptor i)
        {
          if (control_) {
            return alg_->opt_->motionCost(
              alg_->getVertexControls(i, threadId_)->state, goal_->state).value();
          } else {
            return alg_->opt_->motionCost(
              alg_->getVertex(i, threadId_)->state, goal_->state).value();
          }
        }

      private:
        InformedSGCP * alg_{nullptr};
        VertexProperty * goal_{nullptr};
        bool control_{false};
        int threadId_{0};
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
          threadId_(thread_id)
        {
        }
        double operator()(vertex_descriptor i)
        {
          double cost{std::numeric_limits<double>::infinity()};
          // verify that the state is valid
          if (alg_->si_->isValid(alg_->getVertex(i, threadId_)->state)) {
            cost = alg_->getVertex(i, threadId_)->g;
          } else {
            alg_->getVertexMutable(i, threadId_)->blacklisted = true;
          }
          return cost;
        }

      private:
        InformedSGCP * alg_;
        int threadId_{0};
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
        : goalVertex_(goal_vertex),
          numVisits_(num_visits)
        {
        }
        template<class Graph>
        void examine_vertex(Vertex u, Graph & g)
        {
          // check whether examined vertex was goal, if yes throw
          ++(*numVisits_);
          if (u == goalVertex_) {
            throw FoundVertex();
          }
        }

      private:
        Vertex goalVertex_;
        int * numVisits_;
      };  // SimpleVertexVisitor

      /** \brief Keep a global copy of start and goal vertex properties*/
      VertexProperty * startVertexGeometric_{nullptr};
      VertexProperty * goalVertexGeometric_{nullptr};

      std::vector<VertexProperty *> startVerticesControl_;
      std::vector<VertexProperty *> goalVerticesControl_;

      /** \brief The geometric graphs, the numbers of graphs equals to number of threads */
      std::vector<GraphT> graphGeometricThreads_;

      /** \brief The control graphs, the numbers of graphs equals to number of threads */
      std::vector<GraphT> graphControlThreads_;

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

      /** \brief generate a requested amound of states with preffered state sampler
       * \param batch_size number of states to generate
       * \param use_valid_sampler if true, use valid state sampler, otherwise use uniform sampler
       * \param samples vector of states to be filled with generated samples
      */
      void generateBatchofSamples(
        int batch_size,
        bool use_valid_sampler,
        std::vector<ompl::base::State *> & samples);

      /** \brief Keep expanding geometric graph with generated samples, thread-safe.
       * This function is called by each thread in parallel to expand the geometric graphs.
       * For each sample in \e samples, check if it is valid,
       * Check if it is in withing min-max edge range,
       * check if it is unique enough,
       * If all checks pass, add it to the graph and nn structure.
       * Validity check is done with motionCheck, so the edges are collisiosn free.
       * However the user needs to set setLongestValidSegmentFraction() to low value, e.g. 0.01
       * \param samples vector of states to be added to the graph
       * \param ptc termination condition
       * \param geometric_graph the graph to be expanded
       * \param geometric_nn nearest neighbor structure to be updated
       * \param geometric_weightmap weight map to be updated
      */
      void expandGeometricGraph(
        const std::vector<ompl::base::State *> & samples,
        const base::PlannerTerminationCondition & ptc,
        GraphT & geometric_graph,
        std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & geometric_nn,
        WeightMap & geometric_weightmap
      );

      /** \brief After expandGeometricGraph() function call,
       * make sure that the target vertex is connected to it's graph and nearest neighbor.
       * \param target_vertex_property the target vertex to be connected
       * \param geometric_graph the graph to be expanded
       * \param geometric_nn nearest neighbor structure to be updated
       * \param geometric_weightmap weight map to be updated
       * */
      void ensureGeometricGoalVertexConnectivity(
        VertexProperty * target_vertex_property,
        GraphT & geometric_graph,
        std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & geometric_nn,
        WeightMap & geometric_weightmap);

      /** \brief Keep expanding control graph with generated samples.
       * This function is called by each thread in parallel to expand the control graphs.
       * The even threads are used to expand the control graphs from start->goal, e.g. 0,2,...
       * and the odd threads are used to expand the control graphs from goal->start. e.g. 1,3,...
       * However, the even threads tries to connect to odd threads after each sample addition.
       * For instance, the even thread 0 will try to connect to odd thread 1,
       * and the odd thread 1 will try to connect to even thread 0, and so on. This ultimately makes the control search bidirectional.
       * Each thread has its own control graph and nearest neighbor structure.
       * For each sample in \e samples, check if it is valid,
       * Check if it is in withing min-max edge range,
       * check if it is unique enough,
       * If all checks pass, add it to the graph and nn structure.
       * \param samples vector of states to be added to the graph
       * \param target_vertex_state the target vertex state
       * \param target_vertex_descriptor the target vertex descriptor
       * \param ptc termination condition
       * \param connection_control_graph is the graph,  this thread will try to connect to
       * \param connection_control_nn is the nearest neighbor structure, this thread will try to connect to
       * \param control_graph the graph to be expanded (this thread's graph)
       * \param control_nn nearest neighbor structure to be updated (this thread's nn)
       * \param control_weightmap weight map to be updated (this thread's weightmap)
       * \param status is the status of the thread
       */
      void expandControlGraph(
        const std::vector<ompl::base::State *> & samples,
        const ompl::base::State * target_vertex_state,
        const vertex_descriptor & target_vertex_descriptor,
        const base::PlannerTerminationCondition & ptc,
        const GraphT * connection_control_graph,
        const std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> connection_control_nn,
        GraphT & control_graph,
        std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & control_nn,
        WeightMap & control_weightmap,
        int & status);

      /** \brief After expandControlGraph() function call,
       * make sure that the target vertex is connected to it's graph and nearest neighbors if possible.
      * \param target_vertex_state the target vertex state
      * \param target_vertex_descriptor the target vertex descriptor
      * \param control_graph the graph to be expanded (this thread's graph)
      * \param control_nn nearest neighbor structure to be updated (this thread's nn)
      * \param control_weightmap weight map to be updated (this thread's weightmap)
      * \param status is the status of the thread (exact solution ? , unknown ?)
       * */
      void ensureControlGoalVertexConnectivity(
        const ompl::base::State * target_vertex_state,
        const vertex_descriptor & target_vertex_descriptor,
        GraphT & control_graph,
        std::shared_ptr<ompl::NearestNeighbors<VertexProperty *>> & control_nn,
        WeightMap & control_weightmap,
        int & status);

      /** \brief original AIT* function, get number of samples in informed set */
      std::size_t computeNumberOfSamplesInInformedSet() const;

      /** \brief original AIT* function */
      double  computeConnectionRadius(std::size_t numSamples) const;

      /** \brief original AIT* function */
      std::size_t computeNumberOfNeighbors(std::size_t numSamples) const;

      /** \brief compute path cost by finding cost between
       * consecutive vertices in the path
       * \param vertex_path is the path
      */
      ompl::base::Cost computePathCost(
        std::shared_ptr<ompl::control::PathControl> & path) const;

      /** \brief Given a path defined as list of vertex_descriptor \e vertex_path,
       * extract the corresponding states and controls from the graph \e g
       * and populate the ompl::control::PathControl \e path.
       * also populate the vertexprop_path with the vertex properties of the path.
       * \param vertex_path is the path defined as list of vertex_descriptor
       * \param g is the graph from which the path is extracted
       * \param weightmap is the weightmap of the graph from which the path is extracted
       * \param path is the ompl::control::PathControl to be populated
       * \param vertexprop_path is the vector of vertex properties to be populated
       * \param control is a flag to indicate if the path is control path or not
       *  */
      void populateOmplPathfromVertexPath(
        const std::list<vertex_descriptor> & vertex_path,
        GraphT & g,
        WeightMap & weightmap,
        std::shared_ptr<ompl::control::PathControl> & path,
        std::vector<VertexProperty *> & vertexprop_path,
        const bool control = false
      ) const;

      /** \brief Extract the states from the path and return them as a vector of const states
        * \param path is the ompl::control::PathControl from which the states are extracted
        */
      std::vector<const ompl::base::State *> getConstStatesFromPath(
        const std::shared_ptr<ompl::control::PathControl> & path);

      /** \brief Clear the control graphs and the corresponding nearest neighbor structures in all threads
       * but again populate the start and goal vertices in the control graphs while only populating the start vertex in the nearest neighbor structures
        * \param weightmap_controls is the vector of weightmaps of the control graphs
        * \param control_start_goal_descriptors is the vector of start and goal vertex descriptors of the control graphs
       */
      void clearControlGraphs(
        std::vector<WeightMap> & weightmap_controls,
        std::vector<std::pair<vertex_descriptor, vertex_descriptor>> & control_start_goal_descriptors
      );

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

    };      // class InformedSGCP
  }   // namespace control
}   // namespace ompl


#endif  // VOX_NAV_PLANNING__RRT__InformedSGCP_HPP_
