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
	    	// Step 10: get k nearest neighbours
	    	auto knn = find_knn(*q, k);

	    	// Step 11: start inner loop for validity check
	    	for (auto id_iter = knn.begin(); id_iter != knn.end(); id_iter++)
		    {
		    	// Search for Vertex of corresponding ID
		    	auto neighbor_ptr = &configurations.at(id_iter->id);

		    	// Step 12: perform validity check
		    	if (edge_valid(*q, *neighbor_ptr, thresh))
		    	{
		    		// Step 13: add neighbor ID to set of connected edges
		    		Edge qn_edge;
		    		qn_edge.next_id = neighbor_ptr->id;
		    		qn_edge.distance = euclidean_distance(q->coords.x - neighbor_ptr->coords.x,\
				   									  	  q->coords.y - neighbor_ptr->coords.y);
		    		// Add to edges and id_set
		    		q->edges.push_back(qn_edge);
		    		q->id_set.insert(qn_edge.next_id);

		    		// Also do vice versa: add q to neighbor edges
		    		Edge nq_edge;
		    		nq_edge.next_id = q->id;
		    		nq_edge.distance = qn_edge.distance;
		    		// Add to edges and id_set
		    		neighbor_ptr->edges.push_back(nq_edge);
		    		neighbor_ptr->id_set.insert(nq_edge.next_id);
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
				configurations.push_back(q);
			} else {
				// Increment kill counter if there's a collision
				kill_counter++;
			}
		}
	}

	std::vector<Vertex> PRM::find_knn(Vertex & q, const int & k)
	{
		// First, order set in ascending distance from q order
		// copy into vector to use sort fcn
		std::vector<Vertex> configs;
		configs.reserve(configurations.size());
		for (auto q_iter = configurations.begin(); q_iter != configurations.end(); q_iter++)
	    {
	    	configs.push_back(*q_iter);
	    }
		// using lambda for sort criterion. put q in [q] so it can be accessed
		sort(configs.begin(), configs.end(), [q](const Vertex& lhs, const Vertex& rhs)
		{
	      return euclidean_distance(q.coords.x - lhs.coords.x, q.coords.y - lhs.coords.y)\
	      < euclidean_distance(q.coords.x - rhs.coords.x, q.coords.y - rhs.coords.y)\
	      and lhs.id != q.id and rhs.id != q.id;
		});

		// Now, assign IDs of first k elements
		std::vector<Vertex> knn;
		for (int i = 0; i < k; i++)
		{
			knn.push_back(configs.at(i));
		}

		return knn;
	}

	bool PRM::edge_valid(const Vertex & q, const Vertex & q_prime, const double & thresh)
	{
		// Check if New Edge
		if (q.edge_exists(q_prime.id))
		{
			return false;
		} else if (euclidean_distance(q.coords.x - q_prime.coords.x,\
				   q.coords.y - q_prime.coords.y) < thresh)
		// Check Distance Above Useful Threshold
		{
			return false;
		} else if (!no_collision(q, q_prime, inflate_robot))
		// Check if Inflated Robot Intersects Polygon Edge or Path Edge intersects Polygon
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

				// First, check if a potential Vertex is too close to an obstacle Edge
				Vertex A_vtx(A);
				Vertex B_vtx(B);
				if ((too_close(A_vtx, B_vtx, P, inflate_robot)))
				{
					return false;
				}

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

				if (d < 0.0)
					// P is on the right side of at least one edge, not necessarily on obstacle
				{
					on_all_left = false;
				} else if (rigid2d::almost_equal(d, 0.0))
					// if P is on an edge of an obstacle, it is disqualified immediately
				{
					return false;
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
			// int i = static_cast<int>(std::distance(obstacles.begin(), obs_iter));
			// std::cout << "Obstacle #" << i << std::endl;
			if (!no_collision(q, q_prime, obs_iter))
				// Collision! Not a valid Edge, exit here.
			{
				return false;
			} else {
				// Check if inflated robot intersects edge
				for (auto v_iter = obs_iter->vertices.begin(); v_iter != obs_iter->vertices.end(); v_iter++)
				{
					// std::cout << "---------------------------------------------" << std::endl;
					// std::cout << "q: (" << q.coords.x << ", " << q.coords.y << ")" << "\tID: " << q.id << std::endl;
					// std::cout << "q': (" << q_prime.coords.x << ", " << q_prime.coords.y << ")" << "\tID: " << q_prime.id << std::endl;
					// If one Vertex is to close to an Edge, return false and Exit. Otherwise keep checking.
					if (too_close(q, q_prime, Vertex(*v_iter), inflate_robot))
					{
						return false;
					}
				}
			}
		}
		return free;
	}

	bool PRM::too_close(const Vertex & E1, const Vertex & E2, const Vertex & P0, const double & inflate_robot)
	{
		// Referencing: https://docs.google.com/presentation/d/1gQuR4J80aXZ9BBL1s3K83TmxQSWD3AWt/edit#slide=id.g731274b3d5_0_94 Slide 33
		Eigen::Vector2d P1(E1.coords.x, E1.coords.y);
		Eigen::Vector2d P2(E2.coords.x, E2.coords.y);
		Eigen::Vector2d P3(P0.coords.x, P0.coords.y);

		// std::cout << "VTX: (" << P3(0) << ", " << P3(1) << ")" << std::endl;

		double u = ((P3(0) - P1(0)) * (P2(0) - P1(0)) + (P3(1) - P1(1)) * (P2(1) - P1(1)))\
					/ Eigen::Vector2d(P2(0) - P1(0), P2(1) - P1(1)).squaredNorm();

		// std::cout << "u: " << u << std::endl;

		if (u > 0.0 and u < 1.0)
		{
			// the Polygon Vertex is somwhere on the segment, so analysis check is valid
			Eigen::Vector2d closest_point(P1(0) + u * (P2(0) - P1(0)), P1(1) + u * (P2(1) - P1(1)));
			Eigen::Vector2d dist(P3(0) - closest_point(0), P3(1) - closest_point(1));

			// std::cout << "dist: " << dist.norm() << std::endl;

			if (dist.norm() < inflate_robot)
			{
				// The PRM Edge is too close to the Polygon Vertex
				// std::cout << "TOO CLOSE" << std::endl;
				return true;
			}
		}

		return false;
	}

	std::vector<Vertex> PRM::return_prm()
	{
		return configurations;
	}
}