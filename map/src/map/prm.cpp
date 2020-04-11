#include "map/prm.hpp"

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

	void sample_configurations(const int & n)
	{
		// TODO
	}

	std::vector<int> find_knn(const Vertex & q, const int & k)
	{
		// TODO
	}

	bool edge_valid(const Vertex & q, const Vertex & q_prime, const double & thresh)
	{
		// TODO
	}

	bool no_collision(const Vertex & q, const Vertex & q_prime, const double & inflate_robot)
	{
		// TODO
		return true;
	}
}