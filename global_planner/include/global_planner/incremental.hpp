#ifndef INCREMENTAL_INCLUDE_GUARD_HPP
#define INCREMENTAL_INCLUDE_GUARD_HPP
/// \file
/// \brief incremental search library to encompass LPA* and D* Lite planners. I might add Incr. Phi* in the future.

#include "global_planner/heuristic.hpp"

namespace global
{
    // Used to store Obstacle vertex coordinates
    using rigid2d::Vector2D;
    using map::Vertex;
    using map::Cell;
    using map::Index;
    using map::Grid;

    // \brief functor (function object) which compares the priorities of two Nodes for heap sorting 
    class KeyComparator
    { 
    public: 
        int operator() (const Node& n1, const Node& n2) 
        { 
            if (rigid2d::almost_equal(n1.key1, n2.key1))
            {
                return n1.key2 > n2.key2;
            } else
            {
                return n1.key1 > n2.key1;
            }
        } 
    }; 


    // \brief functor (function object) which compares the costs of the predecessor of a Node
    class CostComparator
    { 
    public: 
        int operator() (const Node& n1, const Node& n2) 
        { 
            return n1.gcost > n2.gcost;
        } 
    }; 

    /// \brief LPA* Planner
    class LPAstar : public Astar
    {
    public:
        // Inherit constructor from A*
        using Astar::Astar;

        // NOTE: The method names below directly reference the LPA* pseudocode: http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf

        // \brief Plans an incremental path on a Grid.
        // \returns: the path as a vector of Nodes
        virtual void ComputeShortestPath();

        // Update the cost of a cell and remove it from the open list if it's there
        void UpdateCell(Node & n);

        // \brief sets the g-values of all cells (start and goal for efficiency) to infinity and sets their rhs values according to eqn 1.
        // Also inserts start (locally inconsistent vertex) into the (otherwise empty) priority queue.
        // This guarantees that the first run of ComputeShortestPath performs an exact A* search.
        void Initialize(const Vector2D & start, const Vector2D & goal, const Grid & grid_, const double & resolution);

        // Calculates priorities 1 and 2 of a node, used in sorting for LPA*
        // \param n: the node whose keys we calculate
        void CalculateKeys(Node & n);

        // \brief Termination condition for ComputeShortestPath method.
        bool Continue(const int & iterations);

        // \brief traces the most up-to-date path
        std::vector<Node> trace_path(const Node & final_node);

        // \brief retrieves the 8 neighbours of a node in a grid of Nodes
        // \param n: Node whose neighbours to retrieve
        // \returns: vector of Nodes that are n's neighbours
        std::vector<Node> get_neighbours(const Node & n, const std::vector<Node> & map);

        // \brief returns the most current path
        // \returns: vector of Node
        std::vector<Node> return_path();

    private:
        std::vector<Node> path;
        // Fake grid with limited visibility for simulating increment
        std::vector<Node> FakeGrid;

        // Open list used for finding existing IDs
        std::set<int> open_list_v;
        // Open List
        std::priority_queue <Node, std::vector<Node>, KeyComparator > open_list;

        // For easy management/record keeping
        Node start_node;
        Node goal_node;

        // For tracing most up-to-date path
        Node top_node;

        // Number of visible cells added per increment
        int viz_cells = 1;
    };
}

#endif
