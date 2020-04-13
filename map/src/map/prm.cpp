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
		// MAP EXTENT
		// std::cout << "Map Extent: (" << map_max.x << ", " << map_max.y << ")" << std::endl;
		int kill_counter = 0;
		while (static_cast<int>(configurations.size()) < n or kill_counter > 100 * n)
		{
			// Sample random number with assigned limits
			std::uniform_real_distribution<double> dx(map_min.x, map_max.x);
			std::uniform_real_distribution<double> dy(map_min.y, map_max.y);
			// Random Number Generator defined in nuslam package
			double sample_x = dx(nuslam::get_random());
			double sample_y = dy(nuslam::get_random());

			Vector2D coords(sample_x, sample_y);

			Vertex q(coords);

			// ID = position in vector. eg: (0 elements) -> ID = 0 (first one)
			q.id = static_cast<int>(configurations.size());

			// Ensure to free-space collison
			if (no_collision(q))
			{
				// KEY, OBJECT
				configurations.insert({q.id, q});
			} else {
				// Increment kill counter if there's a collision
				kill_counter++;
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

	bool PRM::no_collision(const Vertex & q)
	{
		bool free = true;
		// Loop over all obstacles
		for (auto obs_iter = obstacles.begin(); obs_iter != obstacles.end(); obs_iter++)
		{
			// Loop over all vertices and treat them as directional vectors.

			// if q is ON ANY edge, then it is on an obstacle

			// If q is on the left side of ALL edges, then it is inside an obstacle
			bool on_all_left = true;

			for (auto v_iter = obs_iter->vertices.begin(); v_iter != obs_iter->vertices.end(); v_iter++)
			{
				// Vector from current and next vertex. so if 3 vertices: 0->1, 1->2, 2->3, 3->0
				// Record current index
				int i = static_cast<int>(std::distance(obs_iter->vertices.begin(), v_iter));
				// If current index is the last one, loop back to zero for vector construction
				if (i == static_cast<int>(obs_iter->vertices.size()) - 1)
				{
					i = 0;
				} else
				// Otherwise, just use the next index
				{
					i += 1;
				}

				// Referencing: https://drive.google.com/file/d/1gQuR4J80aXZ9BBL1s3K83TmxQSWD3AWt/view?usp=sharing Slide 32
				// v_iter is A
				// obs_iter->vertices.at(i) is B
				// DIRECTION is A->B
				// q is P
				auto A = *v_iter;
				auto B = obs_iter->vertices.at(i);
				auto P = q;
				// u is perpendicular to AB. flip xs and ys and negate one component
				Eigen::Vector2d u(-(B.y - A.y), B.x - A.x);
				// dot product with AB = 0 to ensure orthogonal
				Eigen::Vector2d AB(B.x - A.x, B.y - A.y);
				double ABu_test = AB.dot(u);
				// std::cout << "AB u TEST: " << ABu_test << std::endl;
				if (!rigid2d::almost_equal(ABu_test, 0.0))
				{
					throw std::runtime_error("u is NOT orthogonal to AB!\
											 \n  where(): PRM::no_collision(const Vertex &q)");
				}

				// n is the normal vector of u
				auto n = u.normalized(); // .normalize() for in place
				// D is vector A->P
				Eigen::Vector2d D(P.coords.x - A.x, P.coords.y - A.y);
				// Finally, d = D*n (dot product) gives us the minimum distance from AB to P

				// if d > 0, P is on the left of AB, if d = 0, P is ON AB, if d < 0, P is on the right of AB
				double d = D.dot(n);

				// std::cout << "d: " << d << std::endl;

				if (d < 0.0)
					// P is on the right side of at least one edge, not necessarily on obstacle
				{
					on_all_left = false;
				} else if (rigid2d::almost_equal(d, 0.0))
					// if P is on an edge of an obstacle, it is disqualified immediately
				{
					free = false;
				}
			}

			// is q is on the inside of all the edges of an obstacle, it is not free (disqualified)
			if (on_all_left)
			{
				free = false;
			}

		}

		return free;
	}

	bool PRM::no_collision(const Vertex & q, const Vertex & q_prime, const double & inflate_robot)
	{
		// TODO
		return true;
	}

	std::unordered_map<int, Vertex> PRM::return_prm()
	{
		return configurations;
	}
}