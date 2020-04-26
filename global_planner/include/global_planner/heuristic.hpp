#ifndef HEURISTIC_INCLUDE_GUARD_HPP
#define HEURISTIC_INCLUDE_GUARD_HPP
/// \file
/// \brief Heuristic search library to encompass A*, Theta*, Incremental Phi*, D* planners..

#include "global_planner/global_planner.hpp"
#include <queue>
#include <set>

namespace global
{
    // Used to store Obstacle vertex coordinates
    using rigid2d::Vector2D;
    using map::Vertex;
    using map::Cell;
    using map::Index;

    // \brief contains relevant information for heuristic planners
    struct Node
    {
        // Empty constructor
        Node();

        // Stores position and other attributes  wrt PRM map
        Vertex vertex = Vertex(Vector2D());

        // Stores position and other attributes wrt Grid Map
        Cell cell = Cell(Vector2D(), 0.0);

        // row-major index for Grid or connected Vertex ID for PRM
        int parent_id = -1;

        double gcost, hcost, fcost = 0.0;
    };

    // Bolean operator so that std::find can be used with struct sub-element in closed list
    bool operator<(const Node & n, const int & id) { return n.vertex.id < id; }
    bool operator<(const int & id, const Node & n) { return id < n.vertex.id; }
    bool operator<(const Node & n1, const Node & n2) { return n1.vertex.id < n2.vertex.id; }

    // \brief functor (function object) which compares the costs of two Nodes for heap sorting 
    class HeapComparator
    { 
    public: 
        int operator() (const Node& n1, const Node& n2) 
        { 
            if (rigid2d::almost_equal(n1.fcost, n2.fcost))
            {
                return n1.hcost > n2.hcost;
            } else
            {
                return n1.fcost > n2.fcost;
            }
        } 
    }; 

    /// \brief stores Obstacle(s) to construct basic Planner
    class Astar : public GlobalPlanner
    {
    public:
        /// \brief constructor to initialize the A* Planner
        Astar(std::vector<Obstacle> & obstacles_);

        // \brief Plans a path on a PRM.
        // \param start: the starting coordinates
        // \param goal: the goal coordinates
        // \param map: the PRM
        // \returns: the path as a vector of Nodes
        std::vector<Node> plan(Vector2D & start, Vector2D & goal, std::vector<Vertex> & map);

        // \brief returns the path planned on the PRM
        // \param closed_list: Nodes to traverse
        std::vector<Node> trace_path(Node & final_node, std::set<Node, std::less<>> & closed_list);

        // \brief Plans a path on a Grid.
        // \param start: the starting coordinates
        // \param goal: the goal coordinates
        // \param map: the Grid Map
        // \returns: the path as a vector of Nodes
        std::vector<Node> plan(Vector2D & start, Vector2D & goal, std::vector<Cell> & map);

    protected:
        std::vector<Vertex> PRM;

        std::vector<Cell> GRID;

         // Map obstacles
        std::vector<Obstacle> obstacles;

        // Map maximum coordinatesin x,y
        Vector2D map_max;

        // Map minimum coordinatesin x,y
        Vector2D map_min;
    };


    // \brief returns the vertex that most closely matches the given cartesian coordinates
    // \param position: the coordinates
    // \param map: the map whose elements are being searched for coordinates
    Vertex find_nearest_node(Vector2D & position, std::vector<Vertex> & map);
}

#endif
