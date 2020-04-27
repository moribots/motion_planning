#ifndef PRM_INCLUDE_GUARD_HPP
#define PRM_INCLUDE_GUARD_HPP
/// \file
/// \brief PRM Library to build a Probabilistic Roadmap.
#include <map/map.hpp>
#include <unordered_set>
#include <unordered_map>

namespace map
{
    using rigid2d::Vector2D;

    struct Edge
    {
        // ID of next node connected by edge
        // init to -1 for error checking
        int next_id = -1;
        // Euclidean Distance between Nodes (Vertices)
        int distance;
    };

    struct Vertex
    {
        // \brief Constructor with Coordinates
        Vertex(const Vector2D coords_);

        // ID of current Node/Vertex
        // init to -1 for error checking
        int id = -1;
        // Cartesian Coordinates of Vertex
        Vector2D coords;
        // Edges connected to this node
        std::vector<Edge> edges;
        // IDs of Adjacent Nodes
        // O(1) best case or O(n) worst case
        std::unordered_set<int> id_set;

        // Helps with search
        bool visited;

        /// \brief Check if edge exists in ID Hash Table
        /// \param check_id: ID of node to connect to
        /// \returns True if current Vertex connected to given ID.
        bool edge_exists(const int & check_id) const;
    };

    // \brief struct to store important attributes of the shortest distance from a point to a line segment
    struct ShortestDistance
    {
        Vector2D point; // The point whose shortest distance to a line segment is computed.
        double u; // A point is within a line segment if u is between [0,1]
        double D; // The signed shotrest distance distance from the point to the line segment.
        // if D > 0, the point is on the left-hand side of the line segment from Vertex 1 to Vertex 2 
    };

    /// \brief stores Obstacle(s) to construct basic PRM. Inherits from Map in map.hpp.
    class PRM : public Map
    {
        // Inherits Constructors
        using Map::Map;

    public:

        // \brief Collision checking on Concave polygons is very tedious, so we divide them into Convex ones.
        void divide_concave_polygons();

        // \brief Constructs a Roadmap.
        // \param n: number of nodes to put in the Roadmap.
        // \param k: number of closest neighbours to examine for each configuration.
        // \param thresh: Euclidean Distance Threshold for valid Edge.
        void build_map(const int & n, int & k, const double & thresh);

        // \brief Sample free space Q for configurations q. Steps 3-8 of algorithm.
        // \param n: number of nodes to put in the Roadmap.
        void sample_configurations(const int & n);

        // \brief For a given configuration q, assigns indeces of K Nearest Neighbours to q id_set. Step 10 of algorithm.
        // \param q: the Vertex being examined. Not const because id_set is modified.
        // \param k: number of closest neighbours to examine for each configuration.
        // NOTE: Using Brute Force Now, replace with KD-Tree + Rebalance
        std::vector<Vertex> find_knn(Vertex & q, const int & k);

        // \brief Check is the Edge between two nodes is valid (no collision, and above some euclidean distance)
        // \param q: the main Vertex being examined
        // \param q_prime: the second Vertex being examined
        // \param thresh: Euclidean Distance Threshold for valid Edge.
        bool edge_valid(const Vertex & q, const Vertex & q_prime, const double & thresh);

        // \brief Checks whether a potential Edge intersects a Polygon.
        // 'map::PRM::no_collision(const Vertex & q, const Vertex & q_prime, const double & inflate_robot)' calls this function.
        // \param q: the main Vertex being examined
        // \param q_prime: the second Vertex being examined
        // \param obs_iter: iterator for the Obstacle (Polygon) whose vertices we examine.
        bool no_collision(const Vertex & q, const Vertex & q_prime, const std::vector<Obstacle>::iterator & obs_iter);

        // \brief Checks whether a potential Edge intersects an Obstacle considering the robot's geometry.
        // \param q: the main Vertex being examined
        // \param q_prime: the second Vertex being examined
        // \param inflate_robot: approximate robot radius used for collision checking.
        bool no_collision(const Vertex & q, const Vertex & q_prime, const double & inflate_robot);

        // \brief computes the shortest distance from a line to a point and returns the result in an informative struct.
        // \param E1: the first Vertex forming an edge
        // \param E2: the second Vertex forming an edge
        // \param P0: the Vertex whose closeness is being examined.
        // \param inflate_robot: approximate robot radius used for collision checking.
        // \returns ShortestDistance struct
        friend ShortestDistance lineToPoint(const Vertex & E1, const Vertex & E2, const Vertex & P0, const double & inflate_robot);

        // \brief Checks whether a potential Vertex lies on an Obstacle. 'map::PRM::sample_configurations' calls this function.
        // \param q: the Vertex being examined
        friend bool not_inside(const Vertex & q, const std::vector<Obstacle> & obstacles, const double & inflate_robot);

        // \brief Checks if a Vertex is too close to an Edge.
        // \param E1: the first Vertex forming an edge
        // \param E2: the second Vertex forming an edge
        // \param P0: the Vertex whose closeness is being examined.
        // \param inflate_robot: approximate robot radius used for collision checking.
        friend bool too_close(const Vertex & E1, const Vertex & E2, const Vertex & P0, const double & inflate_robot);

        // \brief Return Probabilistic Road Map
        // \returns Probabilistic Road Map 
        std::vector<Vertex> return_prm();
    private:
        // Hash Table
        std::vector<Vertex> configurations;
    };

    // \brief Checks whether a potential Edge intersects a Polygon.
    // 'map::PRM::no_collision(const Vertex & q, const Vertex & q_prime, const double & inflate_robot)' calls this function.
    // \param q: the main Vertex being examined
    // \param q_prime: the second Vertex being examined
    // \param obs_iter: iterator for the Obstacle (Polygon) whose vertices we examine.
    bool no_intersect(const Vertex & q, const Vertex & q_prime, const std::vector<Obstacle>::iterator & obs_iter);

    // \brief Checks whether a potential Vertex lies on an Obstacle. 'map::PRM::sample_configurations' calls this function.
    // \param q: the Vertex being examined
    bool not_inside(const Vertex & q, const std::vector<Obstacle> & obstacles, const double & inflate_robot);

    // \brief Checks if a Vertex is too close to an Edge.
    // \param E1: the first Vertex forming an edge
    // \param E2: the second Vertex forming an edge
    // \param P0: the Vertex whose closeness is being examined.
    // \param inflate_robot: approximate robot radius used for collision checking.
    bool too_close(const Vertex & E1, const Vertex & E2, const Vertex & P0, const double & inflate_robot);

    // \brief computes the shortest distance from a line to a point and returns the result in an informative struct.
    // \param E1: the first Vertex forming an edge
    // \param E2: the second Vertex forming an edge
    // \param P0: the Vertex whose closeness is being examined.
    // \returns ShortestDistance struct
    ShortestDistance lineToPoint(const Vertex & E1, const Vertex & E2, const Vertex & P0);
}

#endif
