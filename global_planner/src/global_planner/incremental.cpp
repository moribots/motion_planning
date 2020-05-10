#include "global_planner/incremental.hpp"

namespace global
{
	void LPAstar::ComputeShortestPath(const Vector2D & start, const Vector2D & goal, const map::Grid & grid_, const double & resolution)
	{

	}


	void LPAstar::UpdateCell(Node & n)
	{
		// Check this node isn't the start node
		if (n.id != start_node.id)
		{
			std::priority_queue <Node, std::vector<Node>, CostComparator > pred_costs;
			// Find the predecessors of node n
			std::vector<Cell> predecessors = get_neighbours(n, GRID);

			for (auto pred_iter = predecessors.begin(); pred_iter < predecessors.end(); pred_iter++)
	    	{
	    		// Find the neighbour node
	    		Node predecessor;
	    		predecessor.cell = *pred_iter;
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
			if (node.id = goal_node.id)
			{
				node = goal_node;
			} else if (node.id = start_node.id)
			{
				node = start_node;
			}
			node.gcost = 1e12;
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


	std::vector<Node> LPAstar::return_path()
	{
		return path;
	}

}	