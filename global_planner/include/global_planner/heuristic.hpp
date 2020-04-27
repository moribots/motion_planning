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

    /// \brief A* Planner
    class Astar : public GlobalPlanner
    {
    public:
        /// \brief constructor to initialize the A* Planner
        Astar(const std::vector<Obstacle> & obstacles_, const double & inflate_robot_);

        // \brief Plans a path on a PRM.
        // \param start: the starting coordinates
        // \param goal: the goal coordinates
        // \param map: the PRM - may be modified if shorter paths are found
        // \returns: the path as a vector of Nodes
        std::vector<Node> plan(const Vector2D & start, const Vector2D & goal, std::vector<Vertex> & map);

        // \brief potentially modify the g cost and parent of a Node. virtual so it can be overriden by Thetastar.
        // \param open_list: list containing nodes to evaluate. Not const because it can be modified
        // \param neighbour: Node in the open list being potentially modified (also not const)
        virtual void update_node(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node);

        // \brief insert a Node into the open list as is defined in the A* method
        // \param open_list: list containing nodes to evaluate. Not const because it can be modified
        // \param neighbour: Node to be added to open list (also not const)
        virtual void create_node(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node);

        // \brief returns the path planned on the PRM
        // \param closed_list: Nodes to traverse
        std::vector<Node> trace_path(const Node & final_node, const std::set<Node, std::less<>> & closed_list);

        // \brief Plans a path on a Grid.
        // \param start: the starting coordinates
        // \param goal: the goal coordinates
        // \param map: the Grid Map
        // \returns: the path as a vector of Nodes
        std::vector<Node> plan(const Vector2D & start, const Vector2D & goal, const std::vector<Cell> & map);
    };

    /// \brief Theta* Planner
    class Thetastar : public Astar
    {
    public:

        // Inherit constructor from A*
        using Astar::Astar;

        // \brief insert a Node into the open list as is defined in the A* method
        // \param open_list: list containing nodes to evaluate. Not const because it can be modified
        // \param neighbour: Node to be added to open list (also not const)
        void create_node(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node) override;

        // \brief Overriden: potentially modify the g cost and parent of a Node. May get parent of parent as parent depending on line of sight
        // \param open_list: list containing nodes to evaluate. Not const because it can be modified
        // \param neighbour: Node in the open list being potentially modified (also not const)
        void update_node(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node) override;
    };


    // \brief returns the vertex that most closely matches the given cartesian coordinates
    // \param position: the coordinates
    // \param map: the map whose elements are being searched for coordinates
    Vertex find_nearest_node(const Vector2D & position, const std::vector<Vertex> & map);
}

#endif
