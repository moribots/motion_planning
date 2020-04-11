#include "map/prm.hpp"
#include "nuslam/ekf.hpp"  // for random number engine

namespace map
{
	using rigid2d::Vector2D;

	// Vertex
	Vertex::Vertex(const Vector2D coords_)
	{
		coords = coords_;
	}

	bool Vertex::edge_exists(const int & check_id)
	{
		const auto search = id_set.find(check_id);
		return (search == id_set.end()) ? false : true;
	}

	// PRM
	void PRM::build_map(const int & n, const int & k)
	{
		//TODO
	}

	void PRM::sample_configurations(const int & n)
	{
		while (static_cast<int>(configurations.size()) < n)
		{
			std::normal_distribution<double> d(0, 1);
			double sample_x = map_extent.x * d(nuslam::get_random());
			double sample_y = map_extent.y * d(nuslam::get_random());

			Vector2D q(sample_x, sample_y);

			if (no_collision(q, inflate_robot))
			{
				configurations.push_back(q);
			}

		}
	}

	std::vector<int> PRM::find_knn(const Vertex & q, const int & k)
	{
		// TODO
		std::vector<int> i;
		return i;
	}

	bool PRM::edge_valid(const Vertex & q, const Vertex & q_prime, const double & thresh)
	{
		// TODO
		return true;
	}

	bool PRM::no_collision(const Vertex & q, const double & inflate_robot)
	{
		// TODO
		return true;
	}

	bool PRM::no_collision(const Vertex & q, const Vertex & q_prime, const double & inflate_robot)
	{
		// TODO
		return true;
	}
}