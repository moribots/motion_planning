#ifndef MAP_INCLUDE_GUARD_HPP
#define MAP_INCLUDE_GUARD_HPP
/// \file
/// \brief Map Library to store and display polygon-based custom map.
/// NOTE: rigid2d is from turtlebot3_from_scratch (tb3 in nuturtle.rosinstall)
#include <rigid2d/rigid2d.hpp>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace map
{
    // Used to store Obstacle vertex coordinates
    using rigid2d::Vector2D;

    struct Obstacle
    /// \brief stores obstacle vertex coordinates in CCW order
    {
        std::vector<Vector2D> vertices;

        // \brief constructor for Obstacle without inputs
        Obstacle();

        // \brief constructor for Obstacle with inputs
        Obstacle(const std::vector<Vector2D> & vertices_);
    };

    /// \brief stores Obstacle(s) to construct basic map
    class Map
    {
    public:
        /// \brief the default constructor creates an empty map
        Map();

        /// \brief this constructor creates a map that holds user-specified obstacles
        /// \param obstacles_: the list of obstacles in this map
        Map(const std::vector<Obstacle> & obstacles_);

        /// \brief this constructor creates a map that holds user-specified obstacles and robot size threshold.
        /// \param obstacles_: the list of obstacles in this map
        // \param inflate_robot_: approximate robot radius used for collision checking.
        Map(const std::vector<Obstacle> & obstacles_, const double inflate_robot_);

        /// \brief return vector of obstacles
        /// \returns vector of Obstacle
        std::vector<Obstacle> return_obstacles();

        /// \brief assigns the map space to map_extent in absolute(x,y).
        void find_map_extent();

        // \brief return map bounds
        // \returns std::vector<Vector2D> containing minimum and maximum bounds respectively
        std::vector<Vector2D> return_map_bounds();


    // protected instead of private so that child Class can access
    protected:
        // Map obstacles
        std::vector<Obstacle> obstacles;

        // Map maximum coordinatesin x,y
        Vector2D map_max;

        // Map minimum coordinatesin x,y
        Vector2D map_min;

        // approximate robot radius used for collision checking.
        double inflate_robot;
    };

    double euclidean_distance(const double & x_rel, const double & y_rel);
}

#endif
