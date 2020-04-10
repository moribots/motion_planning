#ifndef MAP_INCLUDE_GUARD_HPP
#define MAP_INCLUDE_GUARD_HPP
/// \file
/// \brief Map Library to store and display polygon-based custom map.
/// NOTE: rigid2d is from turtlebot3_from_scratch (tb3 in nuturtle.rosinstall)
#include <rigid2d/rigid2d.hpp>
#include <vector>

namespace map
{
    // Used to store Obstacle vertex coordinates
    using rigid2d::Vector2D;

    struct Obstacle
    /// \brief stores obstacle vertex coordinates in CCW order
    {
        std::vector<Vector2D> vertices;

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

        /// \brief return vector of obstacles
        /// \returns vector of Obstacle
        std::vector<Obstacle> return_obstacles();

    private:
        // Map obstacles
        std::vector<Obstacle> obstacles;
    };
}

#endif
