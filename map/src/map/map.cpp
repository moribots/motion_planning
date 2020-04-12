#include "map/map.hpp"

namespace map
{
	using rigid2d::Vector2D;

	// Obstacle
	Obstacle::Obstacle()
	{
	}

	Obstacle::Obstacle(const std::vector<Vector2D> & vertices_)
	{
		vertices = vertices_;
	}

	// Map
	Map::Map()
	{
		find_map_extent();
		inflate_robot = 0.5;
	}

	Map::Map(const std::vector<Obstacle> & obstacles_)
	{
		obstacles = obstacles_;

		find_map_extent();

		inflate_robot = 0.5;
	}

	Map::Map(const std::vector<Obstacle> & obstacles_, const double inflate_robot_)
	{
		obstacles = obstacles_;

		find_map_extent();

		inflate_robot = inflate_robot_;
	}

	std::vector<Obstacle> Map::return_obstacles()
	{
		return obstacles;
	}

	void Map::find_map_extent()
	{
		double x_min, x_max, y_min, y_max = 0;

		for (auto obs_iter = obstacles.begin(); obs_iter != obstacles.end(); obs_iter++)
	    {

			for (auto v_iter = obs_iter->vertices.begin(); v_iter != obs_iter->vertices.end(); v_iter++)
		    {
		    	if (v_iter->x > x_max)
		    	{
		    		x_max = v_iter->x;
		    	} else if (v_iter->x < x_min)
		    	{
		    		x_min = v_iter->x;
		    	}

		    	if (v_iter->y > y_max)
		    	{
		    		y_max = v_iter->y;
		    	} else if (v_iter->y < y_min)
		    	{
		    		y_min = v_iter->y;
		    	}

		    	map_max = Vector2D(x_max, y_max);
		    	map_min = Vector2D(x_min, y_min);
		    }
		}
	}


	// Helper Functions
	double euclidean_distance(const double & x_rel, const double & y_rel)
	{
		return sqrt(pow(x_rel, 2) + pow(y_rel, 2));
	}

}