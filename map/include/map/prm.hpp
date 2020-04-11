#ifndef PRM_INCLUDE_GUARD_HPP
#define PRM_INCLUDE_GUARD_HPP
/// \file
/// \brief PRM Library to build a Probabilistic Roadmap.
/// NOTE: rigid2d is from turtlebot3_from_scratch (tb3 in nuturtle.rosinstall)
#include <rigid2d/rigid2d.hpp>
#include <map/map.hpp>
#include <vector>

namespace map
{
    using rigid2d::Vector2D;

    struct Edge
    {
        // ID of node connected by edge
        int next_id = -1;
        // Euclidean Distance between Nodes (Vertices)
        int distance
    };

    struct Vertex
    {
        // ID of current Node/Vertex
        int id = -1;
        // Cartesian Coordinates of Vertex
        Vector2D coord;
        // Edges connected to this node
        std::vector<Edge> edges;
        // IDs of Adjacent Nodes - HASH TABLE
        // O(1) best case or O(n) worst case
        std::unordered_set<int> id_set;

        bool visited

        /// \brief Check if edge exists in ID Hash Table
        bool edgeExists(int id);
    }

    /// \brief stores Obstacle(s) to construct basic PRM. Inherits from Map in map.hpp.
    class PRM public Map
    {
    public:
        void obstacles_to_hash

    private:
    };
}

#endif
