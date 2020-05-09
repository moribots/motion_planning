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

    /// \brief LPA* Planner
    class LPAstar : public Astar
    {
    public:
        // Inherit constructor from A*
        using Astar::Astar;

        // NOTE: The method names below directly reference the LPA* pseudocode: http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf

        // \brief Plans an incremental path on a Grid.
        // \param start: the starting coordinates
        // \param goal: the goal coordinates
        // \param map: the Grid Map
        // \returns: the path as a vector of Nodes
        virtual void ComputeShortestPath(const Vector2D & start, const Vector2D & goal, const map::Grid & grid_, const double & resolution);

        // Update the cost of a cell and remove it from the open list if it's there
        void UpdateCell(const Node & n, std::priority_queue <Node, std::vector<Node>, KeyComparator > & open_list, std::set<int> & open_list_v);

        // \brief sets the g-values of all cells (start and goal for efficiency) to infinity and sets their rhs values according to eqn 1.
        // Also inserts start (locally inconsistent vertex) into the (otherwise empty) priority queue.
        // This guarantees that the first run of ComputeShortestPath performs an exact A* search.
        void Initialize(const Vector2D & start, const Vector2D & goal, std::priority_queue <Node, std::vector<Node>, KeyComparator > & open_list,
                        std::set<int> & open_list_v);

        // Calculates priorities 1 and 2 of a node, used in sorting for LPA*
        // \param n: the node whose keys we calculate
        void CalculateKeys(Node & n);

        // \brief returns the most current path
        // \returns: vector of Node
        std::vector<Node> return_path();

    private:
        std::vector<Node> path;
    };
}

#endif
