#include "map/map.hpp"

namespace map
{
	using rigid2d::Vector2D;

	// Obstacle
	Obstacle::Obstacle(const std::vector<Vector2D> & vertices_)
	{
		vertices = vertices_;
	}

	// Map
	Map::Map(const std::vector<Obstacle> & obstacles_)
	{
		obstacles = obstacles_;
	}

	std::vector<Obstacle> Map::return_obstacles()
	{
		return obstacles;
	}

}