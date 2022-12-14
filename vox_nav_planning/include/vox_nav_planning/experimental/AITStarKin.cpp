/**
 * Example use of boost::astar_search_no_init on an infinite, implicitly-defined graph.
 *
 * The graph type used here is XYZGraph, representing an infinite grid of squares.  Each
 * square is connected to its eight neighbors; however, the example shows how to use
 * boost::filtered_graph to make the search take place only along orthogonal edges.
 */

#include <iostream>
#include <list>
#include <map>
#include <set>
#include <utility>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/operators.hpp>
#include <boost/ref.hpp>

namespace Direction
{
  enum id
  {
    MIN = 0,
    N = MIN, S, E, W, NW, NE, SE, SW, NONE
  };
}

struct XYZ : public boost::additive<XYZ, boost::totally_ordered<XYZ, boost::equivalent<XYZ>>>
{
  typedef int X;
  typedef int Y;
  typedef int Z;

  XYZ(X x = 0, Y y = 0, Z z = 0);

  // Same square counts.
  bool adjacentTo(XYZ const & that) const;

  XYZ & operator=(XYZ const & that);
  XYZ & operator+=(XYZ const & that);

  bool operator<(XYZ const & that) const;

  X x;
  Y y;
  Z z;

  XYZ neighbor(Direction::id direction) const;
  std::set<XYZ> allNeighbors() const;
};

std::ostream & operator<<(std::ostream & os, XYZ const & xyz);

struct neighbor_iterator;

/*
 * Model of:
 *  * Graph
 *  * IncidenceGraph
 */
struct XYZGraph
{
  XYZGraph();

  // Graph concept requirements
  typedef XYZ vertex_descriptor;
  typedef std::pair<XYZ, XYZ> edge_descriptor;
  typedef boost::undirected_tag directed_category;
  typedef boost::disallow_parallel_edge_tag edge_parallel_category;
  typedef boost::incidence_graph_tag traversal_category;

  // IncidenceGraph concept requirements
  typedef neighbor_iterator out_edge_iterator;
  typedef int degree_size_type;
};

namespace boost
{
  template<>
  struct graph_traits<XYZGraph>
  {
    typedef XYZGraph G;

    typedef G::vertex_descriptor vertex_descriptor;
    typedef G::edge_descriptor edge_descriptor;
    typedef G::out_edge_iterator out_edge_iterator;

    typedef G::directed_category directed_category;
    typedef G::edge_parallel_category edge_parallel_category;
    typedef G::traversal_category traversal_category;

    typedef G::degree_size_type degree_size_type;

    typedef void in_edge_iterator;
    typedef void vertex_iterator;
    typedef void vertices_size_type;
    typedef void edge_iterator;
    typedef void edges_size_type;
  };
}

// IncidenceGraph concept requirements
std::pair<XYZGraph::out_edge_iterator, XYZGraph::out_edge_iterator> out_edges(
  XYZGraph::vertex_descriptor v, XYZGraph const & g);
XYZGraph::degree_size_type out_degree(XYZGraph::vertex_descriptor v, XYZGraph const & g);
XYZGraph::vertex_descriptor source(XYZGraph::edge_descriptor e, XYZGraph const & g);
XYZGraph::vertex_descriptor target(XYZGraph::edge_descriptor e, XYZGraph const & g);

// Iterator
struct neighbor_iterator
  : public boost::iterator_facade<neighbor_iterator,
    std::pair<XYZ, XYZ>,
    boost::forward_traversal_tag,
    std::pair<XYZ, XYZ>>
{
public:
  neighbor_iterator();
  neighbor_iterator(XYZ xyz, Direction::id direction);

  neighbor_iterator & operator=(neighbor_iterator const & that);

  std::pair<XYZ, XYZ> operator*() const;
  neighbor_iterator & operator++();
  bool operator==(neighbor_iterator const & that) const;

  bool equal(neighbor_iterator const & that) const {return operator==(that);}
  void increment() {operator++();}

private:
  XYZ xyz;
  Direction::id direction;
};


// Filter used to traverse grid only along orthogonal (non-diagonal) edges.
struct orthogonal_only
{
  typedef std::pair<XYZ, XYZ> Edge;
  bool operator()(Edge const & edge) const
  {
    return edge.first.x == edge.second.x || edge.first.y == edge.second.y;
  }
};

template<typename Graph>
class distance_heuristic;

struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(XYZ goal)
  : m_goal(goal) {}

  void examine_vertex(XYZ xyz, XYZGraph const & g)
  {
    (void)g;
    std::cout << "Exploring " << xyz << "..." << std::endl;
    if (xyz == m_goal) {
      throw found_goal();
    }
  }
  void examine_vertex(XYZ xyz, boost::filtered_graph<XYZGraph, orthogonal_only> const & g)
  {
    (void)g;
    std::cout << "Exploring " << xyz << "..." << std::endl;
    if (xyz == m_goal) {
      throw found_goal();
    }
  }

private:
  XYZ m_goal;
};

template<typename K, typename V>
class default_map
{
public:
  typedef K key_type;
  typedef V data_type;
  typedef std::pair<K, V> value_type;

  default_map(V const & defaultValue)
  : m()
    , defaultValue(defaultValue)
  {}

  V & operator[](K const & k)
  {
    if (m.find(k) == m.end()) {
      m[k] = defaultValue;
    }
    return m[k];
  }

private:
  std::map<K, V> m;
  V const defaultValue;
};

struct PredecessorMap
{
  PredecessorMap()
  : m() {}
  PredecessorMap(PredecessorMap const & that)
  : m(that.m) {}

  typedef XYZ key_type;
  typedef XYZ value_type;
  typedef XYZ & reference_type;
  typedef boost::read_write_property_map_tag category;

  XYZ & operator[](XYZ xyz) {return m[xyz];}

  std::map<XYZ, XYZ> m;
};

XYZ get(PredecessorMap const & pm, XYZ xyz)
{
  std::map<XYZ, XYZ>::const_iterator found = pm.m.find(xyz);
  return (found != pm.m.end()) ? found->second : xyz;
}

void put(PredecessorMap & pm, XYZ key, XYZ value)
{
  pm.m[key] = value;
}

// Euclidean distance heuristic (square root omitted)
template<typename Graph>
class distance_heuristic : public boost::astar_heuristic<Graph, int>
{
public:
  distance_heuristic(XYZ goal)
  : m_goal(goal) {}
  unsigned operator()(XYZ xyz)
  {
    int dx = m_goal.x - xyz.x;
    int dy = m_goal.y - xyz.y;
    unsigned retval = static_cast<unsigned>(dx * dx + dy * dy);
    return retval;
  }

private:
  XYZ m_goal;
};

int main()
{
  XYZGraph baseGraph;
  boost::filtered_graph<XYZGraph, orthogonal_only> g(baseGraph, orthogonal_only());
  //BOOST_CONCEPT_ASSERT((IncidenceGraphConcept< boost::filtered_graph<XYZGraph, orthogonal_only> >));


  XYZ start(0, 0, 8);
  XYZ goal(5, 7, 0);

  std::cout << "Start vertex: " << start << std::endl;
  std::cout << "Goal vertex: " << goal << std::endl;

  PredecessorMap p;
  typedef boost::associative_property_map<default_map<XYZ, unsigned>> DistanceMap;
  typedef default_map<XYZ, unsigned> WrappedDistanceMap;
  WrappedDistanceMap wrappedMap = WrappedDistanceMap(std::numeric_limits<unsigned>::max());
  wrappedMap[start] = 0;
  DistanceMap d = DistanceMap(wrappedMap);
  auto weight_map = default_map<std::pair<XYZ, XYZ>, unsigned>(1);
  auto vertex_index_map = std::map<XYZ, unsigned>();
  auto rank_map = std::map<XYZ, unsigned>();
  auto color_map = std::map<XYZ, boost::default_color_type>();

  try {
    astar_search_no_init(
      g,
      start,
      distance_heuristic<XYZGraph>(goal),
      visitor(astar_goal_visitor(goal))
      .distance_map(d)
      .predecessor_map(boost::ref(p))
      .weight_map(
        boost::associative_property_map<default_map<std::pair<XYZ, XYZ>,
        unsigned>>(weight_map))
      .vertex_index_map(boost::associative_property_map<std::map<XYZ, unsigned>>(vertex_index_map))
      .rank_map(boost::associative_property_map<std::map<XYZ, unsigned>>(rank_map))
      .color_map(
        boost::associative_property_map<std::map<XYZ, boost::default_color_type>>(
          color_map))
      .distance_compare(std::less<unsigned>())
      .distance_combine(std::plus<unsigned>())
    );
  } catch (found_goal const &) { // found a path to the goal
    std::list<XYZ> shortest_path;
    for (XYZ xyz = goal;; xyz = p[xyz]) {
      shortest_path.push_front(xyz);
      if (p[xyz] == xyz) {
        break;
      }
    }
    std::cout << "Shortest path from " << start << " to "
              << goal << ": ";
    std::list<XYZ>::iterator spi = shortest_path.begin();
    std::cout << start;
    for (++spi; spi != shortest_path.end(); ++spi) {
      std::cout << " -> " << (*spi);
    }
    std::cout << std::endl;
    return 0;
  }

  std::cout << "Didn't find a path from " << start << "to"
            << goal << "!" << std::endl;
  return 0;
}

XYZGraph::XYZGraph()
{}

std::pair<XYZGraph::out_edge_iterator, XYZGraph::out_edge_iterator>
out_edges(
  XYZGraph::vertex_descriptor v,
  XYZGraph const & g)
{
  (void)g;
  return std::make_pair(
    XYZGraph::out_edge_iterator(v, Direction::MIN),
    XYZGraph::out_edge_iterator(v, Direction::NONE) );
}

XYZGraph::degree_size_type
out_degree(
  XYZGraph::vertex_descriptor v,
  XYZGraph const & g)
{
  (void)g;
  return v.allNeighbors().size();
}

XYZGraph::vertex_descriptor
source(
  XYZGraph::edge_descriptor e,
  XYZGraph const & g)
{
  (void)g;
  return e.first;
}

XYZGraph::vertex_descriptor target(
  XYZGraph::edge_descriptor e,
  XYZGraph const & g)
{
  (void)g;
  return e.second;
}

neighbor_iterator::neighbor_iterator()
: xyz(), direction() {}

neighbor_iterator::neighbor_iterator(XYZ xyz, Direction::id direction)
: xyz(xyz)
  , direction(direction)
{
}

neighbor_iterator & neighbor_iterator::operator=(neighbor_iterator const & that)
{
  xyz = that.xyz;
  direction = that.direction;
  return *this;
}

std::pair<XYZ, XYZ> neighbor_iterator::operator*() const
{
  std::pair<XYZ, XYZ> const retval = std::make_pair(xyz, xyz.neighbor(direction));
  return retval;
}

neighbor_iterator & neighbor_iterator::operator++()
{
  direction = static_cast<Direction::id>(int(direction) + 1);
  return *this;
}

bool neighbor_iterator::operator==(neighbor_iterator const & that) const
{
  return xyz == that.xyz && direction == that.direction;
}


XYZ::XYZ(X x, Y y, Z z)
: x(x)
  , y(y)
  , z(z)
{
}

bool XYZ::adjacentTo(XYZ const & that) const
{
  return abs(x - that.x) <= 1 && abs(y - that.y) <= 1;
}

XYZ & XYZ::operator=(XYZ const & that)
{
  x = that.x;
  y = that.y;
  z = that.z;
  return *this;
}

XYZ & XYZ::operator+=(XYZ const & that)
{
  x += that.x;
  y += that.y;
  z += that.z;
  return *this;
}

bool XYZ::operator<(XYZ const & that) const
{
  return x < that.x || (x == that.x && y < that.y);
}

std::ostream & operator<<(std::ostream & os, XYZ const & xyz)
{
  os << "(" << xyz.x << "," << xyz.y << "," << xyz.z << ")";
  return os;
}

XYZ XYZ::neighbor(Direction::id direction) const
{
  using namespace Direction;

  int dx = 0, dy = 0;
  switch (direction) {
    case NW:
    case W:
    case SW:
      dx = -1;
      break;
    case NE:
    case E:
    case SE:
      dx = 1;
      break;
    default:
      dy = 0;
  }
  switch (direction) {
    case NW:
    case N:
    case NE:
      dy = -1;
      break;
    case SW:
    case S:
    case SE:
      dy = 1;
      break;
    default:
      dy = 0;
  }
  XYZ const neighbor(x + dx, y + dy);
  return neighbor;
}

std::set<XYZ> XYZ::allNeighbors() const
{
  std::set<XYZ> neighbors;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      neighbors.insert(XYZ(x + dx, y + dy));
    }
  }

  return neighbors;
}
