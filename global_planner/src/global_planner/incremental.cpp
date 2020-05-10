#include "global_planner/incremental.hpp"

namespace global
{
	void LPAstar::ComputeShortestPath(const Vector2D & start, const Vector2D & goal, const map::Grid & grid_, const double & resolution)
	{
		int iterations= 0;
		while (Continue())
		{
			iterations++;

		}

	}


	void LPAstar::UpdateCell(Node & n)
	{
		// Check this node isn't the start node
		if (n.id != start_node.id)
		{
			std::priority_queue <Node, std::vector<Node>, CostComparator > pred_costs;
			// Find the predecessors of node n
			std::vector<Node> predecessors = get_neighbours(n, FakeGrid);

			for (auto pred_iter = predecessors.begin(); pred_iter < predecessors.end(); pred_iter++)
	    	{
	    		// Find the neighbour node
	    		Node predecessor;
	    		predecessor = *pred_iter;
	    		predecessor.id = predecessor.cell.index.row_major;

	    		// Skip Occupied or Inflated Cells
	    		if (predecessor.cell.celltype == map::Occupied or\
	    			predecessor.cell.celltype == map::Inflation)
	    		{	    			
	    			continue;
	    		} else
	    		// Free Cells
	    		{
	    			// Push to priority queue for auto sort
	    			pred_costs.push(predecessor);
	    		}

	    	}

	    	Node min_predecessor = pred_costs.top();

	    	// Update RHS value
	    	n.rhs = min_predecessor.gcost;
	    	// Update Parent
	    	n.parent_id = min_predecessor.id;


	    	// If it's on the open list, remove it
	    	bool opened = open_list_v.find(n.id) != open_list_v.end();
	    	open_list_v.erase(n.id);
	    	if (opened)
	    	{
	    		std::priority_queue <Node, std::vector<Node>, KeyComparator > temp_open_list;
				while (!open_list.empty())
				{
					Node temp = open_list.top();
					// Don't include the current node since we are trying to erase it
					if (temp.id != n.id)
					{
						temp_open_list.push(temp);
					}
					open_list.pop();
				}

				open_list = temp_open_list;
	    	}

	    	// If n is locally inconsistent (ie if n.gcost != n.rhs), add it to the open list with updated keys
	    	if (!(rigid2d::almost_equal(n.gcost, n.rhs)))
	    	{
	    		CalculateKeys(n);
	    		open_list.push(n);
	    	}
		}

	}


	void LPAstar::Initialize(const Vector2D & start, const Vector2D & goal, const map::Grid & grid_, const double & resolution)
	{
		GRID = grid_.return_grid();
		FakeGrid.clear();

		// Find Start and Goal Nodes
	    goal_node.cell = find_nearest_node(goal, grid_, resolution);
	    goal_node.id = goal_node.cell.index.row_major;
	    CalculateKeys(goal_node);

	    // Find GRID cell whose coordinates most closely match the start coordinates
	    start_node.cell = find_nearest_node(start, grid_, resolution);
	    start_node.id = start_node.cell.index.row_major;
	    start_node.hcost = heuristic(start_node, goal_node);
	    start_node.rhs = 0.0;
	    CalculateKeys(start_node);

		// Populate Fake Grid
		for (auto iter = GRID.begin(); iter < GRID.end(); iter++)
		{
			Node node;
			node.cell = *iter;
			node.id = node.cell.index.row_major;
			if (node.id == goal_node.id)
			{
				node = goal_node;
			} else if (node.id == start_node.id)
			{
				node = start_node;
			}
			node.gcost = 1e12;
			// TODO: When vanilla is done, clear all obstacle/inflated in FakeGrid for simulated increment
			FakeGrid.push_back(node);
		}

		// Populate Open List
		open_list.push(start_node);
		open_list_v.insert(start_node.id);
	}


	void LPAstar::CalculateKeys(Node & n)
	{
		n.key1 = std::min(n.gcost, n.rhs + n.hcost);
		n.key2 = std::min(n.gcost, n.rhs);
	}


	bool LPAstar::Continue()
	{
		Node top = open_list.top();
		CalculateKeys(goal_node);

		// Put top and goal_node in a priority queue to see which has the smallest key
		std::priority_queue <Node, std::vector<Node>, KeyComparator > check;

		check.push(top);
		check.push(goal_node);

		// FIRST condition
		if (check.top().id != goal_node.id)
		{
			return true;
		} else if (goal_node.rhs != goal_node.gcost)
		// SECOND condition
		{
			return true;
		} else
		{
			return false;
		}
	}


	std::vector<Node> LPAstar::get_neighbours(const Node & n, const std::vector<Node> & map)
	{
		int x_max = map.back().cell.index.x;
		int y_max = map.back().cell.index.y; 
		std::vector<Node> neighbours;
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
						Node nbr = map.at(rmj);
						neighbours.push_back(nbr);
					}
				}
			}
		}
		return neighbours;
	}


	std::vector<Node> LPAstar::return_path()
	{
		return path;
	}

}	