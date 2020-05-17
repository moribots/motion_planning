#include "global_planner/heuristic.hpp"

namespace global
{
	Node::Node()
	{
		vertex = Vertex(Vector2D());
		cell = Cell(Vector2D(), 0.0);
	}

	Astar::Astar(const std::vector<Obstacle> & obstacles_, const double & inflate_robot_)
	{
		obstacles = obstacles_;
		inflate_robot = inflate_robot_;
	}

	// PRM version
	std::vector<Node> Astar::plan(const Vector2D & start, const Vector2D & goal, std::vector<Vertex> & map)
	{
		PRM = map;

		/**
			Main Planning Loop
            for A* (naive)

            Steps:

            1. Append start node to open list
            2. LOOP:
                a- set current node to that with lowest f cost
                in open list. If some nodes have the same f cost,
                choose current node using h cost

                b- get the 8 neighbours of each node (or less depending
                on proximity to grid limits)

                c- for each node:
                    i - if it is inside the closed list,
                    or if it is an obstacle, ignore it
                    ii - if it is on the open list, check
                    whether it is a better path than the current node,
                    if so, update its g cost and make its parent node the
                    current node
                    iii - if none of these conditions are met,
                    add the node to the open list and set its parent
                    to the current node after calculating its g and h cost
            3. If goal is found, return path, else, keep looping
            until goal is found or open list is 0 (path was never found)
        **/

        // Create a Min heap of Nodes 
	    std::priority_queue <Node, std::vector<Node>, HeapComparator > open_list;

	    // Store the goal node
	    Node goal_node;
	    // Find PRM vertex whose coordinates most closely match the goal coordinates
	    goal_node.vertex = find_nearest_node(goal, PRM);
	    goal_node.id = goal_node.vertex.id;

	    // Add the start node to the queue
	    Node current_node;
	    // Find PRM vertex whose coordinates most closely match the start coordinates
	    current_node.vertex = find_nearest_node(start, PRM);
	    current_node.id = current_node.vertex.id;

	    open_list.push(current_node);

	    // For re-ordering and finding NOTE: inefficient
	    std::set<int> open_list_v;
	    open_list_v.insert(current_node.id);

	    // Store the start node
	    Node start_node = current_node;


	    // Closed List
	    std::set<Node, std::less<>> closed_list;

	    int iterations = 0;
	    while (!open_list.empty())
	    {
	    	iterations++;

	    	// Get the minimum node on the open list
	    	current_node = open_list.top();
	    	// Remove said node from open list
	    	open_list.pop();
	    	open_list_v.erase(current_node.id);

	    	// Add current node ID to closed list
	    	closed_list.insert(current_node);

	    	// END condition
	    	if (current_node.vertex.id == goal_node.vertex.id)
	    	{
	    		std::cout << "Goal found after " << iterations << " Iterations!" << std::endl;
	    		return trace_path(current_node, closed_list);
	    	}

	    	// Loop through each node's neighbors
	    	for (auto edge_iter = current_node.vertex.edges.begin(); edge_iter < current_node.vertex.edges.end(); edge_iter++)
	    	{
	    		// Skip if neighbour in closed list, or is obstacle (GRID) 
	    		bool skip = false;
	    		// Modify node cost if neighbour but also is in open list
	    		bool opened = false;

	    		// Find the neighbour node
	    		Node neighbour;
	    		neighbour.vertex = PRM.at(edge_iter->next_id);
	    		// Self-identify
				neighbour.id = neighbour.vertex.id;

	    		// First, check if in closed list
	    		skip = closed_list.find(neighbour.id) != closed_list.end();
	    		if (skip)
	    		{
	    			continue;
	    		}

	    		// No obstacle list check since not GRID

	    		// Check if in open list
	    		opened = open_list_v.find(neighbour.id) != open_list_v.end();

	    		if (!opened)
	    		// Create a new node and push
	    		{
	    			// hcost is distance from neighbor to goal
	    			neighbour.hcost = map::euclidean_distance(neighbour.vertex.coords.x - goal_node.vertex.coords.x,
	    													  neighbour.vertex.coords.y - goal_node.vertex.coords.y);
	    			
	    			create_vtx(open_list, neighbour, current_node);
	    			// Push to record keeping set
	    			open_list_v.insert(neighbour.id);

	    		} else
	    		// Potentially modify existing node and resort open list
	    		{
	    			update_vtx(open_list, neighbour, current_node);
	    		}
	    	}
	    }

	    // If we have reached this point, then there was no valid path
	    std::cout << "No valid path! returning most complete path" << std::endl;
	    return trace_path(current_node, closed_list);

	}

	void Astar::create_vtx(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node)
	{
		// gcost is current node's gcost + distance from neighbor to current node
		neighbour.gcost = current_node.gcost + map::euclidean_distance(neighbour.vertex.coords.x - current_node.vertex.coords.x,
												  					   neighbour.vertex.coords.y - current_node.vertex.coords.y);
		// Update f cost
		neighbour.fcost = neighbour.gcost + neighbour.hcost;

		// Add parent id
		neighbour.parent_id = current_node.id;

		// Push to open list
		open_list.push(neighbour);
	}

	void Astar::update_vtx(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node)
	{
		// Calculate a new tentative g cost
		double gcost = current_node.gcost + map::euclidean_distance(neighbour.vertex.coords.x - current_node.vertex.coords.x,
												  					   neighbour.vertex.coords.y - current_node.vertex.coords.y);
		if (gcost < neighbour.gcost)
		// Modify Node
		{
			// Update g cost
			neighbour.gcost = gcost;
			// Update f cost
			neighbour.fcost = neighbour.gcost + neighbour.hcost;
			// Update parent
			neighbour.parent_id = current_node.id;

			// If this happens, we need to re-sort the open list
			std::priority_queue <Node, std::vector<Node>, HeapComparator > temp_open_list;
			while (!open_list.empty())
			{
				Node temp = open_list.top();
				if (temp.id == neighbour.id)
				{
					temp = neighbour;
				}
				temp_open_list.push(temp);
				open_list.pop();
			}

			open_list = temp_open_list;
		}

	}

	std::vector<Node> Astar::trace_path(const Node & final_node, const std::set<Node, std::less<>> & closed_list)
	{
		// First node in the vector is 'final node'
		std::vector<Node> path{final_node};

		bool done = false;

		while (!done)
		{
			// std::cout << "GOING FROM NODE: " << path.back().vertex.id << " TO: " << path.back().parent_id << std::endl;
			Node next_node = *closed_list.find(path.back().parent_id);
			path.push_back(next_node);
			if (next_node.parent_id == -1)
			{
				done = true;
				break;
			}
		}

		std::reverse(path.begin(), path.end());

		std::cout << "The path contains " << path.size() << " Nodes." << std::endl;
		return path;
	}


	std::vector<Node> Astar::plan(const Vector2D & start, const Vector2D & goal, const Grid & grid_, const double & resolution)
	{
		Grid grid = grid_;
		GRID = grid.return_grid();

		/**
			Main Planning Loop
            for A* (naive)

            Steps:

            1. Append start node to open list
            2. LOOP:
                a- set current node to that with lowest f cost
                in open list. If some nodes have the same f cost,
                choose current node using h cost

                b- get the 8 neighbours of each node (or less depending
                on proximity to grid limits)

                c- for each node:
                    i - if it is inside the closed list,
                    or if it is an obstacle, ignore it
                    ii - if it is on the open list, check
                    whether it is a better path than the current node,
                    if so, update its g cost and make its parent node the
                    current node
                    iii - if none of these conditions are met,
                    add the node to the open list and set its parent
                    to the current node after calculating its g and h cost
            3. If goal is found, return path, else, keep looping
            until goal is found or open list is 0 (path was never found)
        **/

        // Create a Min heap of Nodes 
	    std::priority_queue <Node, std::vector<Node>, HeapComparator > open_list;

	    // Store the goal node
	    Node goal_node;
	    // Find GRID cellwhose coordinates most closely match the goal coordinates
	    goal_node.cell = find_nearest_node(goal, grid, resolution);
	    goal_node.id = goal_node.cell.index.row_major;

	    // Add the start node to the queue
	    Node current_node;
	    // Find GRID cell whose coordinates most closely match the start coordinates
	    current_node.cell = find_nearest_node(start, grid, resolution);
	    current_node.id = current_node.cell.index.row_major;
	    // std::cout << "START IDX: [" << current_node.cell.index.x << ", " << current_node.cell.index.y << "]" << std::endl;

	    // std::cout << "GOAL IDX: [" << goal_node.cell.index.x << ", " << goal_node.cell.index.y << "]" << std::endl;

	    open_list.push(current_node);

	    // For re-ordering and finding NOTE: inefficient
	    std::set<int> open_list_v;
	    open_list_v.insert(current_node.id);

	    // Store the start node
	    Node start_node = current_node;


	    // Closed List
	    std::set<Node, std::less<>> closed_list;

	    int iterations = 0;
	    while (!open_list.empty())
	    {
	    	iterations++;

	    	// Get the minimum node on the open list
	    	current_node = open_list.top();
	    	// Remove said node from open list
	    	open_list.pop();
	    	open_list_v.erase(current_node.id);

	    	// Add current node ID to closed list
	    	closed_list.insert(current_node);

	    	// END condition
	    	if (current_node.cell.index.row_major == goal_node.cell.index.row_major)
	    	{
	    		std::cout << "Goal found after " << iterations << " Iterations!" << std::endl;
	    		return trace_path(current_node, closed_list);
	    	}

	    	// Find the current node's neighbours
	    	std::vector<Cell> neighbours = get_neighbours(current_node, GRID);

	    	// Loop through each node's neighbors
	    	for (auto nbr_iter = neighbours.begin(); nbr_iter < neighbours.end(); nbr_iter++)
	    	{
	    		// Skip if neighbour in closed list, or is obstacle (GRID) 
	    		bool skip = false;
	    		// Modify node cost if neighbour but also is in open list
	    		bool opened = false;

	    		// Find the neighbour node
	    		Node neighbour;
	    		neighbour.cell = *nbr_iter;
	    		neighbour.id = neighbour.cell.index.row_major;

	    		// First, check if in closed list
	    		skip = closed_list.find(neighbour.id) != closed_list.end();
	    		if (skip)
	    		{
	    			// std::cout << "CLOSED Node at [" << neighbour.cell.index.x << ", " << neighbour.cell.index.y << "]" << "\t [" << neighbour.cell.coords.x << ", " << neighbour.cell.coords.y << "]" << std::endl;
	    			continue;
	    		}

	    		// Now, check if neighbour is an obstacle
	    		if (neighbour.cell.celltype == map::Occupied or\
	    			neighbour.cell.celltype == map::Inflation)
	    		{	    			
	    			continue;
	    		}

	    		// Check if in open list
	    		opened = open_list_v.find(neighbour.id) != open_list_v.end();

	    		if (!opened)
	    		// Create a new node and push
	    		{
	    			neighbour.hcost = heuristic(neighbour, goal_node);
	    			// std::cout << "h cost: " << neighbour.hcost << std::endl;
	    			
	    			create_cell(open_list, neighbour, current_node);
	    			// Push to record keeping set
	    			open_list_v.insert(neighbour.id);

	    		} else
	    		// Potentially modify existing node and resort open list
	    		{
	    			update_cell(open_list, neighbour, current_node);
	    		}
	    	}
	    }

	    // If we have reached this point, then there was no valid path
	    std::cout << "No valid path! returning most complete path" << std::endl;
	    return trace_path(current_node, closed_list);

	}

	void Astar::update_cell(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node)
	{
		// Calculate a new tentative g cost
		double gcost = current_node.gcost + heuristic(neighbour, current_node);
		if (gcost < neighbour.gcost)
		// Modify Node
		{
			// Update g cost
			neighbour.gcost = gcost;
			// Update f cost
			neighbour.fcost = neighbour.gcost + neighbour.hcost;
			// Update parent
			neighbour.parent_id = current_node.id;

			// If this happens, we need to re-sort the open list
			std::priority_queue <Node, std::vector<Node>, HeapComparator > temp_open_list;
			while (!open_list.empty())
			{
				Node temp = open_list.top();
				if (temp.id == neighbour.id)
				{
					temp = neighbour;
				}
				temp_open_list.push(temp);
				open_list.pop();
			}

			open_list = temp_open_list;
		}

	}

	void Astar::create_cell(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node)
	{
		// gcost is current node's gcost + distance from neighbor to current node
		neighbour.gcost = current_node.gcost + heuristic(neighbour, current_node);
		// std::cout << "g cost: " << neighbour.gcost << std::endl;
		// Update f cost
		neighbour.fcost = neighbour.gcost + neighbour.hcost;

		// Add parent id
		neighbour.parent_id = current_node.id;

		// Push to open list
		open_list.push(neighbour);
	}

	double Astar::heuristic(const Node & n1, const Node & n2)
	{
		double x_dist = fabs(n1.cell.center_coords.x - n2.cell.center_coords.x);
		double y_dist = fabs(n1.cell.center_coords.y - n2.cell.center_coords.y);
		// return map::euclidean_distance(x_dist, y_dist);

		double D1 = 1.0;
		double D2 = sqrt(2.0);

		return D1 * (x_dist + y_dist) + (D2 - 2 * D1) * std::min(x_dist, y_dist);
	}


	std::vector<Cell> Astar::get_neighbours(const Node & n, const std::vector<Cell> & map)
	{
		int x_max = map.back().index.x;
		int y_max = map.back().index.y; 
		std::vector<Cell> neighbours;
		// std::cout << "Node at [" << n.cell.index.x << ", " << n.cell.index.y << "]" << std::endl;

		// Evaluate about 3x3 block for 8-connectivity
		for (int x = -1; x < 2; x++)
		{
			for (int y = -1; y < 2; y++)
			{
				// Skip x,y = (0,0) since that's the current node
				if (x == 0 and y == 0)
				{
					continue;
				} else
				{
					int check_x = n.cell.index.x + x;
					int check_y = n.cell.index.y + y;

					// Ensure potential neighbour is within grid bounds
					if (check_x >= 0 and check_x <= x_max and check_y >= 0 and check_y <= y_max)
					{
						// Now we need to grab the right cell from the map. To do this: index->RMJ
						// std::cout << "Neighbour at [" << check_x << ", " << check_y << "]" << std::endl;
						int rmj = map::grid2rowmajor(check_x, check_y, x_max + 1);
						Cell nbr = map.at(rmj);
						neighbours.push_back(nbr);
					}
				}
			}
		}
		return neighbours;
	}


	void Thetastar::create_vtx(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node)
	{
		// First, do line of sight check between PARENT of current node and neighbour
		bool clear = true;

		int grandparent_id = current_node.parent_id;
		if (grandparent_id == -1)
		{
			// There is no grandparent since this is the start node
			grandparent_id = current_node.id;
			clear = false;
		}

		if (clear)
		{
			for (auto obs_iter = obstacles.begin(); obs_iter != obstacles.end(); obs_iter++)
			{
				// Edge on Edge Check
				if (!no_intersect(neighbour.vertex, PRM.at(grandparent_id), obs_iter))
				{
					clear = false;
					break;
				}

				// Edge Near Point Check
				for (auto v_iter = obs_iter->vertices.begin(); v_iter != obs_iter->vertices.end(); v_iter++)
				{
					if (too_close(neighbour.vertex, PRM.at(grandparent_id), Vertex(*v_iter), inflate_robot))
					{
						clear = false;
						break;
					}
				}

				// No need to loop over other obstacles if not clear
				if (!clear)
				{
					break;
				}
			}

		}

		if (!clear)
		// Perform vanilla update from A*
		{
			// Use UNMODIFIED inherited function
			Astar::create_vtx(open_list, neighbour, current_node);

		} else
		{
			// g cost of grandparent = current_node gcost - dist(grandparent->current)
			double grandparent_gcost = current_node.gcost - map::euclidean_distance(current_node.vertex.coords.x - PRM.at(grandparent_id).coords.x,
													  					            current_node.vertex.coords.y - PRM.at(grandparent_id).coords.y);

			// g cost is grandparent node g cost + dist(grandparent -> neighbor)
			neighbour.gcost = grandparent_gcost + map::euclidean_distance(neighbour.vertex.coords.x - PRM.at(grandparent_id).coords.x,
													  					neighbour.vertex.coords.y - PRM.at(grandparent_id).coords.y);
			// Update f cost
			neighbour.fcost = neighbour.gcost + neighbour.hcost;

			// Add parent id
			neighbour.parent_id = grandparent_id;

			// Push to open list
			open_list.push(neighbour);

		}
	}


	// The overridden update_vtx fcn with line of sight check
	void Thetastar::update_vtx(std::priority_queue <Node, std::vector<Node>, HeapComparator > & open_list, Node & neighbour, const Node & current_node)
	{
		// First, do line of sight check between PARENT of current node and neighbour
		bool clear = true;

		int grandparent_id = current_node.parent_id;
		if (grandparent_id == -1)
		{
			// There is no grandparent since this is the start node
			grandparent_id = current_node.id;
			clear = false;
		}

		if (clear)
		{
			for (auto obs_iter = obstacles.begin(); obs_iter != obstacles.end(); obs_iter++)
			{
				// Edge on Edge Check
				if (!no_intersect(neighbour.vertex, PRM.at(grandparent_id), obs_iter))
				{
					clear = false;
					break;
				}

				// Edge Near Point Check
				for (auto v_iter = obs_iter->vertices.begin(); v_iter != obs_iter->vertices.end(); v_iter++)
				{
					if (too_close(neighbour.vertex, PRM.at(grandparent_id), Vertex(*v_iter), inflate_robot))
					{
						clear = false;
						break;
					}
				}

				// No need to loop over other obstacles if not clear
				if (!clear)
				{
					break;
				}
			}

		}

		if (!clear)
		// Perform vanilla update from A*
		{
			// Use UNMODIFIED inherited function
			Astar::update_vtx(open_list, neighbour, current_node);

		} else
		// Perform cost update with new parent
		{
			// g cost of grandparent = current_node gcost - dist(grandparent->current)
			double grandparent_gcost = current_node.gcost - map::euclidean_distance(current_node.vertex.coords.x - PRM.at(grandparent_id).coords.x,
													  					            current_node.vertex.coords.y - PRM.at(grandparent_id).coords.y);
			// Calculate a new tentative g cost
			double gcost = grandparent_gcost + map::euclidean_distance(neighbour.vertex.coords.x - PRM.at(grandparent_id).coords.x,
													  					neighbour.vertex.coords.y - PRM.at(grandparent_id).coords.y);
			if (gcost < neighbour.gcost)
			// Modify Node
			{
				// Update g cost
				neighbour.gcost = gcost;
				// Update f cost
				neighbour.fcost = neighbour.gcost + neighbour.hcost;
				// Update parent with PARENT of parent
				// std::cout << "PARENT ID: " << current_node.vertex.id << std::endl;
				neighbour.parent_id = grandparent_id;
				// std::cout << "GRANDPARENT (NEW PARENT) ID: " << neighbour.parent_id << std::endl;

				// If this happens, we need to re-sort the open list
				std::priority_queue <Node, std::vector<Node>, HeapComparator > temp_open_list;
				while (!open_list.empty())
				{
					Node temp = open_list.top();
					if (temp.id == neighbour.id)
					{
						temp = neighbour;
					}
					temp_open_list.push(temp);
					open_list.pop();
				}

				open_list = temp_open_list;
			}
		}
	}

	Vertex find_nearest_node(const Vector2D & position, const std::vector<Vertex> & map)
	{
		double min_dist = map::euclidean_distance(position.x - map.at(0).coords.x, position.y - map.at(0).coords.y);
		int min_idx = 0;

		for (auto iter = map.begin(); iter < map.end(); iter++)
		{
			double curr_dist = map::euclidean_distance(position.x - iter->coords.x, position.y - iter->coords.y);
			if (curr_dist < min_dist)
			{
				min_idx = static_cast<int>(std::distance(map.begin(), iter));
				min_dist = curr_dist;
			}
		}

		return map.at(min_idx);
	}


	Cell find_nearest_node(const Vector2D & position, const Grid & grid, const double & resolution)
	{
		Cell temp_cell(position, resolution);

		Index idx = grid.world2grid(temp_cell);

		return (grid.return_grid().at(idx.row_major));
	}
}