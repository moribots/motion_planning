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

	bool Vertex::edge_exists(const int & check_id) const
	{
		const auto search = id_set.find(check_id);
		return (search == id_set.end()) ? false : true;
	}

	// PRM
	void PRM::build_map(const int & n, const int & k, const double & thresh)
	{
		// Following PRM Algorithm from Lavalle - Planning Algorithms

		// Steps 1-2, empty Vertex and Edge List
		configurations.clear();

		// Steps 3-8: Sample Configurations
		sample_configurations(n);

		// Step 9: start loop for validity check
		for (auto q = configurations.begin(); q != configurations.end(); q++)
	    {
	    	// Step 10: get k nearest neighbours. Key = First | Value = Second
	    	find_knn(q->second, k);

	    	// Step 11: start inner loop for validity check
	    	for (auto id_iter = q->second.id_set.begin(); id_iter != q->second.id_set.end(); id_iter++)
		    {
		    	// Search hash table for Vertex of corresponding ID. Returns iterator of <KEY,VALUE>
		    	auto neighbor_iter = configurations.find(*id_iter);

		    	// Step 12: perform validity check
		    	if (edge_valid(q->second, neighbor_iter->second, thresh))
		    	{
		    		// Step 13: add neighbour ID to set of connected edges
		    		Edge edge;
		    		edge.next_id = neighbor_iter->second.id;
		    		edge.distance = euclidean_distance(q->second.coords.x - neighbor_iter->second.coords.x,\
				   									   q->second.coords.y - neighbor_iter->second.coords.y);
		    		q->second.edges.push_back(edge);
		    	}
		    }
	    }
	}

	void PRM::sample_configurations(const int & n)
	{
		while (static_cast<int>(configurations.size()) < n)
		{
			std::normal_distribution<double> d(0, 1);
			double sample_x = map_extent.x * d(nuslam::get_random());
			double sample_y = map_extent.y * d(nuslam::get_random());

			Vector2D coords(sample_x, sample_y);

			Vertex q(coords);

			// ID = position in vector. eg: (0 elements) -> ID = 0 (first one)
			q.id = static_cast<int>(configurations.size());

			if (no_collision(q, inflate_robot))
			{
				// KEY, OBJECT
				configurations.insert({q.id, q});
			}
		}
	}

	void PRM::find_knn(Vertex & q, const int & k)
	{
		// First, order set in ascending distance from q order
		// copy into vector to use sort fcn
		std::vector<Vertex> configs;
		configs.reserve(configurations.size());
		for (auto q_iter = configurations.begin(); q_iter != configurations.end(); q_iter++)
	    {
	    	configs.push_back(q_iter->second);
	    }
		// using lambda for sort criterion. put q in [q] so it can be accessed
		sort(configs.begin(), configs.end(), [q](const Vertex& lhs, const Vertex& rhs)
		{
	      return euclidean_distance(q.coords.x - lhs.coords.x, q.coords.y - lhs.coords.y)\
	      < euclidean_distance(q.coords.x - rhs.coords.x, q.coords.y - rhs.coords.y)\
	      and lhs.id != q.id and rhs.id != q.id;
		});

		// Now, assign IDs of first k elements
		for (int i = 0; i < k; i++)
		{
			q.id_set.insert(configs.at(i).id);
		}
	}

	bool PRM::edge_valid(const Vertex & q, const Vertex & q_prime, const double & thresh)
	{
		// Check Edge Collision
		if (!no_collision(q, q_prime, inflate_robot))
		{
			return false;
		// Check if New Edge
		} else if (q.edge_exists(q_prime.id))
		{
			return false;
		// Check Distance Above Useful Threshold
		} else if (euclidean_distance(q.coords.x - q_prime.coords.x,\
				   q.coords.y - q_prime.coords.y) < thresh)
		{
			return false;
		} else
		{
			return true;
		}
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