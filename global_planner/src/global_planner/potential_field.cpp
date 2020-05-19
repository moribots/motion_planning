#include "global_planner/potential_field.hpp"

namespace global
{
	using rigid2d::Vector2D;

	// Global Planner
	PotentialField::PotentialField(const std::vector<Obstacle> & obstacles_, const double & eta_, const double & ada_,
                       			   const double & zeta_, const double & d_thresh_, const double & Q_thresh_)
	{
		obstacles = obstacles_;
		eta = eta_;
		zeta = zeta_;
		ada = ada_;
		d_thresh = d_thresh_;
		Q_thresh = Q_thresh_;
	}

	Vector2D PotentialField::AttractiveGradient(const Vector2D & cur_pos, const Vector2D & goal)
	{
		Vector2D dUA;

		double d_goal = map::euclidean_distance(cur_pos.x - goal.x, cur_pos.y - goal.y);

		if (d_goal <= d_thresh or rigid2d::almost_equal(d_goal, d_thresh))
		{
			// Quadratic
			dUA.x = zeta * (cur_pos.x - goal.x);
			dUA.y = zeta * (cur_pos.y - goal.y);
		} else
		{
			// Conic
			dUA.x = d_thresh * zeta * (cur_pos.x - goal.x) / d_goal;
			dUA.y = d_thresh * zeta * (cur_pos.y - goal.y) / d_goal;
		}

		return dUA;
	}

	Vector2D PotentialField::RepulsiveGradient(const Vector2D & cur_pos, const std::vector<Obstacle> & obs)
	{
		Vector2D dUR(0.0, 0.0);

		for (auto obs_iter = obs.begin(); obs_iter < obs.end(); obs_iter++)
		{
			Vector2D closest_point = FindClosestPoint(cur_pos, obs_iter->vertices);

			double Q_obs = map::euclidean_distance(cur_pos.x - closest_point.x, cur_pos.y - closest_point.y);

			if (Q_obs <= Q_thresh or rigid2d::almost_equal(Q_obs, Q_thresh))
			// Closest point within range of influence
			{
				double deltaDx = (cur_pos.x - closest_point.x) / Q_obs;
				double deltaDy = (cur_pos.y - closest_point.y) / Q_obs;

				dUR.x += ada * ((1.0 / Q_thresh) - (1.0 / Q_obs)) * (1.0 / (std::pow(Q_obs, 2))) * deltaDx;
				dUR.y += ada * ((1.0 / Q_thresh) - (1.0 / Q_obs)) * (1.0 / (std::pow(Q_obs, 2))) * deltaDy;
			}
		}

		return dUR;
	}

	Vector2D PotentialField::FindClosestPoint(const Vector2D & cur_pos, const std::vector<Vector2D> & vertices)
	{
		map::Vertex P0(cur_pos);

		double shortest_dist = 1e12;

		Vector2D closest_point(1e12, 1e12);

		for (auto v_iter = vertices.begin(); v_iter < vertices.end(); v_iter++)
		{
			// Vector from current and next vertex. so if 4 vertices: 0->1, 1->2, 2->3, 3->0
			// Record current index
			int i = static_cast<int>(std::distance(vertices.begin(), v_iter));
			// If current index is the last one, loop back to zero for vector construction
			// -1 because indeces start at 0
			if (i == static_cast<int>(vertices.size()) - 1)
			{
				i = 0;
			} else
			// Otherwise, just use the next index
			{
				i += 1;
			}

			// Referencing: https://drive.google.com/file/d/1gQuR4J80aXZ9BBL1s3K83TmxQSWD3AWt/view?usp=sharing Slide 28
			auto E1 = map::Vertex(*v_iter);
			auto E2 = map::Vertex(vertices.at(i));

			map::ShortestDistance shrt = map::lineToPoint(E1, E2, P0);

			if (std::abs(shrt.D) < shortest_dist)
			{
				closest_point = shrt.point;
				shortest_dist = std::abs(shrt.D);
			}
		}

		return closest_point;
	}

	double PotentialField::MultiDimNorm(const std::vector<double> partials)
	{
		double norm_squared = 0.0;

		for (auto iter = partials.begin(); iter < partials.end(); iter++)
		{
			norm_squared += std::pow(*iter, 2);
		}

		return std::sqrt(norm_squared);
	}

	Vector2D PotentialField::OneStepGD(const Vector2D & cur_pos, const Vector2D & goal, std::vector<Obstacle> & obs)
	{
		Vector2D dUA = AttractiveGradient(cur_pos, goal);
		Vector2D dUR = RepulsiveGradient(cur_pos, obs);

		Vector2D dU(dUA.x + dUR.x, dUA.y + dUR.y);

		std::vector<double> partials{dU.x, dU.y};

		double norm = MultiDimNorm(partials);

		double Descent_x = dU.x / norm;
		double Descent_y = dU.y / norm;

		Vector2D new_pos(cur_pos.x - Descent_x * eta, cur_pos.y - Descent_y * eta);

		return new_pos;
	}
}
