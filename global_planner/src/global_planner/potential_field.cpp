#include "global_planner/potential_field.hpp"

namespace global
{
	using rigid2d::Vector2D;

	// Global Planner
	PotentialField::PotentialField(const std::vector<Obstacle> & obstacles_, const double & eta_, const double & zeta_)
	{
		obstacles = obstacles_;
		eta = eta_;
		zeta = zeta_;
	}
}
