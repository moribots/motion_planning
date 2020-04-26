#include "global_planner/heuristic.hpp"

namespace global
{
	Node::Node()
	{
		vertex = Vertex(Vector2D());
		cell = Cell(Vector2D(), 0.0);
	}

	Astar::Astar(std::vector<Obstacle> & obstacles_)
	{
		obstacles = obstacles_;
	}

	// PRM version
	std::vector<Node> Astar::plan(Vector2D & start, Vector2D & goal, std::vector<Vertex> & map)
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

	    // Add the start node to the queue
	    Node current_node;
	    // Find PRM vertex whose coordinates most closely match the start coordinates
	    current_node.vertex = find_nearest_node(start, PRM);

	    open_list.push(current_node);

	    // For re-ordering and finding NOTE: inefficient
	    std::set<int> open_list_v;
	    open_list_v.insert(current_node.vertex.id);

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
	    	open_list_v.erase(current_node.vertex.id);

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

	    		// First, check if in closed list
	    		skip = closed_list.find(neighbour.vertex.id) != closed_list.end();
	    		if (skip)
	    		{
	    			continue;
	    		}

	    		// No obstacle list check since not GRID

	    		// Check if in open list
	    		opened = open_list_v.find(neighbour.vertex.id) != open_list_v.end();

	    		if (!opened)
	    		// Create a new node and push
	    		{
	    			// hcost is distance from neighbor to goal
	    			neighbour.hcost = map::euclidean_distance(neighbour.vertex.coords.x - goal_node.vertex.coords.x,
	    													  neighbour.vertex.coords.y - goal_node.vertex.coords.y);
	    			// gcost is current node's gcost + distance from neighbor to current node
	    			neighbour.gcost = current_node.gcost + map::euclidean_distance(neighbour.vertex.coords.x - current_node.vertex.coords.x,
	    													  					   neighbour.vertex.coords.y - current_node.vertex.coords.y);
	    			// Update f cost
	    			neighbour.fcost = neighbour.gcost + neighbour.hcost;

	    			// Add parent id
	    			neighbour.parent_id = current_node.vertex.id;

	    			// Push to open list
	    			open_list.push(neighbour);
	    			open_list_v.insert(neighbour.vertex.id);

	    		} else
	    		// Potentially modify existing node and resort open list
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
	    				neighbour.parent_id = current_node.vertex.id;

	    				// If this happens, we need to re-sort the open list
	    				std::priority_queue <Node, std::vector<Node>, HeapComparator > temp_open_list;
	    				while (!open_list.empty())
	    				{
	    					temp_open_list.push(open_list.top());
	    					open_list.pop();
	    				}

	    				open_list = temp_open_list;
	    			}
	    		}
	    	}
	    }

	    // If we have reached this point, then there was no valid path
	    std::cout << "No valid path! returning start node" << std::endl;
	    return std::vector<Node>{start_node};

	}

	std::vector<Node> Astar::trace_path(Node & final_node, std::set<Node, std::less<>> & closed_list)
	{
		// First node in the vector is 'final node'
		std::vector<Node> path{final_node};

		Node next_node = *closed_list.find(path.back().parent_id);

		path.push_back(next_node);

		bool done = false;

		while (!done)
		{
			next_node = *closed_list.find(path.back().parent_id);
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

	Vertex find_nearest_node(Vector2D & position, std::vector<Vertex> & map)
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
}