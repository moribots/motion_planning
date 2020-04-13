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
	void PRM::build_map(const int & n, int & k, const double & thresh)
	{
		if (k > n)
		{
			std::cout << "[WARNING]: k > n. We have set k = n." << std::endl;
			k = n;
		}
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
		    		std::cout << "VALID EDGE: " << "CONNECTING " << q->second.id << " AND " << neighbor_iter->second.id << std::endl;
		    		// Step 13: add neighbour ID to set of connected edges
		    		Edge qn_edge;
		    		qn_edge.next_id = neighbor_iter->second.id;
		    		qn_edge.distance = euclidean_distance(q->second.coords.x - neighbor_iter->second.coords.x,\
				   									  	  q->second.coords.y - neighbor_iter->second.coords.y);
		    		q->second.edges.push_back(qn_edge);

		    		// Also do vice versa: add q to neighbour edges
		    		Edge nq_edge;
		    		nq_edge.next_id = q->second.id;
		    		nq_edge.distance = qn_edge.distance;

		    		neighbor_iter->second.edges.push_back(nq_edge);
		    	}
		    }
	    }
	}

	void PRM::sample_configurations(const int & n)
	{
		// MAP EXTENT
		// std::cout << "Map Extent: (" << map_max.x << ", " << map_max.y << ")" << std::endl;
		int kill_counter = 0;
		while (static_cast<int>(configurations.size()) < n or kill_counter > 1000 * n)
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
		// Check Distance Above Useful Threshold
		if (euclidean_distance(q.coords.x - q_prime.coords.x,\
				   q.coords.y - q_prime.coords.y) < thresh)
		{
			return false;
		} else if (!no_collision(q, q_prime, inflate_robot))
		// Check if Inflated Robot Intersects Polygon Edge or Path Edge intersects Polygon
		{
			return false;

		// } else if (q.edge_exists(q_prime.id))
		// // Check if New Edge
		// {
		// 	return false;

		} else 
		{
			// std::cout << "q: (" << q.coords.x << ", " << q.coords.y << ")" << std::endl;
			// std::cout << "q': (" << q_prime.coords.x << ", " << q_prime.coords.y << ")" << std::endl;
			// std::cout << "FREE" << std::endl;
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
				// Vector from current and next vertex. so if 4 vertices: 0->1, 1->2, 2->3, 3->0
				// Record current index
				int i = static_cast<int>(std::distance(obs_iter->vertices.begin(), v_iter));
				// If current index is the last one, loop back to zero for vector construction
				// -1 because indeces start at 0
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
				// NOTE: if we have {x,y} --> {y, -x} = RHS perp. (outward normal) | {-y, x} = LHS perp. (inward normal)
				// inward normal
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

	bool PRM::no_collision(const Vertex & q, const Vertex & q_prime, const std::vector<Obstacle>::iterator & obs_iter)
	{
		// Initialize tE/tL: the max/min entering/leaving segment parameters.
		// NOTE: these evolve over time for each obstacle.
		double tE = 0.0;
		double tL = 1.0;
		// Initialize dS: P0->P1 vector 
		Eigen::Vector2d dS(q_prime.coords.x - q.coords.x, q_prime.coords.y - q.coords.y);
		// Loop over all vertices and treat them as directional Edge vectors.
		for (auto v_iter = obs_iter->vertices.begin(); v_iter != obs_iter->vertices.end(); v_iter++)
		{
			// Vector from current and next vertex. so if 4 vertices: 0->1, 1->2, 2->3, 3->0
			// Record current index
			int i = static_cast<int>(std::distance(obs_iter->vertices.begin(), v_iter));
			// If current index is the last one, loop back to zero for vector construction
			// -1 because indeces start at 0
			if (i == static_cast<int>(obs_iter->vertices.size()) - 1)
			{
				i = 0;
			} else
			// Otherwise, just use the next index
			{
				i += 1;
			}

			// Referencing: https://drive.google.com/file/d/1gQuR4J80aXZ9BBL1s3K83TmxQSWD3AWt/view?usp=sharing Slide 28
			auto Vi = *v_iter;
			auto Vip1 = obs_iter->vertices.at(i);
			// edge i is Vi->Vi+1
			Eigen::Vector2d ei(Vip1.x - Vi.x, Vip1.y - Vi.y);

			// ni is the outward normal of the edge ei
			// NOTE: if we have {x,y} --> {y, -x} = RHS perp. (outward normal) | {-y, x} = LHS perp. (inward normal)
			// outward normal
			Eigen::Vector2d ni(Vip1.y - Vi.y, -(Vip1.x - Vi.x));
			if (!rigid2d::almost_equal(ei.dot(ni), 0.0))
			{
				throw std::runtime_error("ni is NOT orthogonal to ei!\
										 \n  where(): PRM::no_collision(const Vertex &q, const Vertex & q_prime)");
			}
			double N = - ni.dot(Eigen::Vector2d(q.coords.x - Vi.x, q.coords.y - Vi.y));
			double D = ni.dot(dS);

			if (rigid2d::almost_equal(D, 0.0))
			{
				// Then q->q' is parallel to edge ei. Double condition to catch numerical errors
				if (N < 0.0 and !(rigid2d::almost_equal(N, 0.0)))
				{
					// Then q is outside of polygon Obstacle.
					return true; // Intersection not possible. Exit here.
				} else {
					// q->q' cannot enter/leave across edge ei. Process next edge
					continue;
				}
			}
			// Intersection time
			double t = N / D;

			if (D < 0.0)
			{
				// Then segment q->q' enters across edge ei
				tE = std::max(tE, t);
				if (tE > tL)
				{
					// Then the segment enters AFTER leaving. Intersection not possible on this edge.
					return true;
				}

			} else {
				// D > 0 in this case
				// Then segment q->q' leaves across edge ei
				tL = std::min(tL, t);
				if (tL < tE)
				{
					// Then the segment leaves BEFORE entering. Intersection not possible on this edge.
					return true;
				}
			}
		}
		// tE <= tL if we have not exited at an earlier edge.
		// There is a valid intersection. Exit here. No need to check additional obstacles.
		// Entering Point: P(tE) = q + tE * dS
		// Leaving Point: P(tL) =  q + tL * dS
		return false;
	}

	bool PRM::no_collision(const Vertex & q, const Vertex & q_prime, const double & inflate_robot)
	{
		bool free = true;
		// Loop over all obstacles
		for (auto obs_iter = obstacles.begin(); obs_iter != obstacles.end(); obs_iter++)
		{
			int i = static_cast<int>(std::distance(obstacles.begin(), obs_iter));
			// std::cout << "Obstacle #" << i << std::endl;
			if (!no_collision(q, q_prime, obs_iter))
				// Collision! Not a valid Edge, exit here.
			{
				return false;
			} else {
				// Check if inflated robot intersects edge
				// TODO
			}
		}
		// std::cout << "q: (" << q.coords.x << ", " << q.coords.y << ")" << "ID: " << q.id << std::endl;
		// std::cout << "q': (" << q_prime.coords.x << ", " << q_prime.coords.y << ")" << "ID: " << q_prime.id << std::endl;
		// std::cout << "FREE" << std::endl;
		// std::cout << "---------------------------------------------" << std::endl;
		return free;
	}

	std::unordered_map<int, Vertex> PRM::return_prm()
	{
		return configurations;
	}
}