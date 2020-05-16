#include "global_planner/incremental.hpp"

namespace global
{
	void LPAstar::ComputeShortestPath()
	{
		Node min;
		int iterations= 0;
		while (Continue(iterations))
		{

			iterations++;
			// std::cout << "Iteration: " << iterations << std::endl;

			// if (iterations > 10000)
			// {
			// 	break;
			// }

			min = open_list.top();
			open_list.pop();
			open_list_v.erase(min.id);

			// Check if Overconsistent (start always satisfies this)
			if (min.gcost > min.rhs)
			{
				// std::cout << "Locally Overconsistent!" << std::endl;
				// Update True Cost
				min.gcost = min.rhs;
				// Update min in FakeGrid
				FakeGrid.at(min.id) = min;
				// Check Successors
				std::vector<Node> successors = get_neighbours(min, FakeGrid);
				for (auto succ = successors.begin(); succ < successors.end(); succ++)
				{
					UpdateCell(*succ);
				}

			} else
			{
				// std::cout << "NOT Locally Overconsistent!" << std::endl;
				min.gcost = 1e12;
				FakeGrid.at(min.id) = min;
				// Check Successors
				std::vector<Node> successors = get_neighbours(min, FakeGrid);
				// ALSO check min itself
				successors.push_back(min);
				for (auto succ = successors.begin(); succ < successors.end(); succ++)
				{
					UpdateCell(*succ);
				}
			}
		}
		path = trace_path(goal_node);
	}


	void LPAstar::UpdateCell(Node & n)
	{
    	// Check this node isn't the start node
    	// std::cout << "NODE EXAMINED at [" << n.cell.index.x << ", " << n.cell.index.y << "]" << "GCOST: " << n.gcost << std::endl;
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
	    			// Push to priority queue for auto sort
	    			// This is not actually stored anywhere, just useful in getting min gcost for n
	    			predecessor.gcost += heuristic(predecessor, n) + 1e12;
	    		} else
	    		// Free Cells
	    		{
	    			// Push to priority queue for auto sort
	    			// This is not actually stored anywhere, just useful in getting min gcost for n
	    			predecessor.gcost += heuristic(predecessor, n);
	    		}

	    		// std::cout << "NBR at [" << predecessor.cell.index.x << ", " << predecessor.cell.index.y << "]" << "GCOST: " << predecessor.gcost << std::endl;

	    		pred_costs.push(predecessor);

	    	}

	    	Node min_predecessor = pred_costs.top();

	    	// Update RHS value
	    	n.rhs = min_predecessor.gcost;
	    	// Update Parent
	    	n.parent_id = min_predecessor.id;
	    }
    	// If it's on the open list, remove it
    	bool opened = open_list_v.find(n.id) != open_list_v.end();
    	if (opened)
    	{
    		open_list_v.erase(n.id);
    		// std::cout << "Erase, IDX: [" << n.cell.index.x << ", " << n.cell.index.y << "]"<< std::endl;
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
    		n.hcost = heuristic(n, goal_node);
    		CalculateKeys(n);
    		open_list.push(n);
    		open_list_v.insert(n.id);
    	}

    	// Update n in FakeGrid
    	FakeGrid.at(n.id) = n;

    	// std::cout << "-------------------------------" << std::endl;
	}


	void LPAstar::Initialize(const Vector2D & start, const Vector2D & goal, const map::Grid & grid_, const double & resolution)
	{
		GRID = grid_.return_fake_grid();
		FakeGrid.clear();

		// Find Start and Goal Nodes
	    goal_node.cell = find_nearest_node(goal, grid_, resolution);
	    goal_node.id = goal_node.cell.index.row_major;
	    goal_node.hcost = 0.0;
	    CalculateKeys(goal_node);

	    // Find GRID cell whose coordinates most closely match the start coordinates
	    start_node.cell = find_nearest_node(start, grid_, resolution);
	    start_node.id = start_node.cell.index.row_major;
	    start_node.hcost = heuristic(start_node, goal_node);
	    start_node.rhs = 0.0;
	    CalculateKeys(start_node);
	    // std::cout << "START, IDX: [" << start_node.cell.index.x << ", " << start_node.cell.index.y << "]"<< std::endl;
	    // std::cout << "GOAL, IDX: [" << goal_node.cell.index.x << ", " << goal_node.cell.index.y << "]"<< std::endl;

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

		std::cout << "Initialized!" << std::endl;
	}


	void LPAstar::CalculateKeys(Node & n)
	{
		n.hcost = heuristic(n, goal_node);
		n.key1 = std::min(n.gcost, n.rhs) + n.hcost;
		n.key2 = std::min(n.gcost, n.rhs);
	}


	bool LPAstar::Continue(const int & iterations)
	{
		Node top = open_list.top();
		// Update Goal Node from Fake Grid
		goal_node = FakeGrid.at(goal_node.id);
		goal_node.hcost = 0.0;
		CalculateKeys(goal_node);

		// Put top and goal_node in a priority queue to see which has the smallest key
		std::priority_queue <Node, std::vector<Node>, KeyComparator > check;

		check.push(top);
		check.push(goal_node);

		// Conditions for Continuing
		if ((check.top().id != goal_node.id) or (!(rigid2d::almost_equal(goal_node.rhs, goal_node.gcost))))
		{
			return true;
		} else
		{
			std::cout << "Goal found after " << iterations << " Iterations!" << std::endl;
			// Reset Goal RHS
			FakeGrid.at(goal_node.id) = goal_node;
			return false;
		}
	}
	

	std::vector<Node> LPAstar::trace_path(const Node & final_node)
	{
		// First node in the vector is 'final node'
		path.clear();
		path.push_back(final_node);

		// std::cout << "START Node at [" << final_node.cell.index.x << ", " << final_node.cell.index.y << "]" << std::endl;

		bool done = false;

		while (!done)
		{
			// std::cout << "GOING FROM NODE: " << path.back().id << " TO: " << path.back().parent_id << std::endl;
			int next_node_id = path.back().parent_id;
			int parent_id = FakeGrid.at(next_node_id).parent_id;
			if (parent_id >= 0)
			{
				int grandparent_id = FakeGrid.at(parent_id).parent_id;
				if (grandparent_id == next_node_id)
					// TWO NODES ARE EACH OTHERS PARENTS.
					// THIS USUALLY MEANS THAT THE GOAL IS INSIDE AN OBSTACLE
				{
					Node n1 = FakeGrid.at(next_node_id);
					Node n2 = FakeGrid.at(parent_id);
					// std::cout << "Two Nodes are each others' parents!" << std::endl;
					// std::cout << "Node 1 at [" << n1.cell.index.x << ", " << n1.cell.index.y << "]" << std::endl;
					// std::cout << "Node 2 at [" << n2.cell.index.x << ", " << n2.cell.index.y << "]" << std::endl;

					if (n1.hcost < n2.hcost or rigid2d::almost_equal(n1.hcost, n2.hcost))
					{
						// The current node is closest to the goal, so return it as the final path
						ROS_WARN("There is no valid path. The goal is in a blocked cell! \n Returning closest path.");
						path.clear();
						path.push_back(n1);
						done = true;
						break;
					} else
					// Go to next node, which will be the closest to the goal
					{
						path.push_back(FakeGrid.at(next_node_id));

						done = true;
						break;
					}
				}
			}

			if (next_node_id < 0 or next_node_id >= static_cast<int>(FakeGrid.size()))
			{
				std::cout << "The Path contains an invalid ID! Returning most complete path!" << std::endl;
				done = true;
				break;
			} else 
			{
				Node next_node = FakeGrid.at(next_node_id);
				if (next_node.cell.celltype == map::Occupied or next_node.cell.celltype == map::Inflation)
				{
					std::cout << "THE PATH CONTAINS AN OBSTACLE! Returning most complete path!" << std::endl;
					done = true;
					break;
				}

				path.push_back(next_node);
				// std::cout << "NEXT Node at [" << next_node.cell.index.x << ", " << next_node.cell.index.y << "]" << std::endl;
				if (next_node.parent_id == -1)
				{
					done = true;
					break;
				}

			}
		}

		std::reverse(path.begin(), path.end());

		std::cout << "The path contains " << path.size() << " Nodes." << std::endl;
		return path;
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
						// std::cout << "Checked Neighbour at [" << nbr.cell.index.x << ", " << nbr.cell.index.y << "]" << std::endl;
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


	std::vector<Node> LPAstar::SimulateUpdate(const std::vector<Cell> & updated_grid)
	{
		GRID = updated_grid;
		std::vector<Node> updated_nodes;
		for (unsigned int i = 0; i < updated_grid.size(); i++)
		{
			// For each UPDATED Cell (cell.newView = true;), UpdateCell() on its neighbours
			if (updated_grid.at(i).newView and FakeGrid.at(i).cell.celltype != updated_grid.at(i).celltype)
			{
				// First, update FakeGrid
				FakeGrid.at(i).cell = updated_grid.at(i);
				// Push back culprit
				updated_nodes.push_back(FakeGrid.at(i));

				// std::cout << "NEW OCCUPANCY: " << FakeGrid.at(i).cell.index.row_major << std::endl;

				std::vector<Node> neighbours = get_neighbours(FakeGrid.at(i), FakeGrid);
				for (auto iter = neighbours.begin(); iter < neighbours.end(); iter++)
				{
					UpdateCell(*iter);
					// Push back neighbours
					updated_nodes.push_back(*iter);
				}
				// Update Changed Node
				UpdateCell(FakeGrid.at(i));
			}
		}

		// Update Goal Node
		UpdateCell(FakeGrid.at(goal_node.id));
		goal_node = FakeGrid.at(goal_node.id);

		// Update Start Node
		UpdateCell(FakeGrid.at(start_node.id));
		start_node = FakeGrid.at(start_node.id);

		// Re-sort open-list
		// std::priority_queue <Node, std::vector<Node>, KeyComparator > temp_open_list;
		// while (!open_list.empty())
		// {
		// 	Node temp = open_list.top();
		// 	temp.hcost = heuristic(temp, goal_node);
		// 	CalculateKeys(temp);
		// 	temp_open_list.push(temp);
		// 	open_list.pop();
		// 	FakeGrid.at(temp.id) = temp;
		// }
		// open_list = temp_open_list;

		if (start_node.cell.celltype == map::Occupied or start_node.cell.celltype == map::Inflation)
		{
			// std::cout << "There is no valid path. Start node is occupied." << std::endl;
			trace_path(goal_node);
			// Return updated nodes
			return updated_nodes;
		}

		// Compute Shortest Path
		ComputeShortestPath();
		
		// Return updated nodes
		return updated_nodes;
	}


	void DSL::Initialize(const Vector2D & start, const Vector2D & goal, const map::Grid & grid_, const double & resolution)
	{
		GRID = grid_.return_fake_grid();
		FakeGrid.clear();

		// NOTE: START AND GOAL POSITIONS FLIPPED FOR D*LITE
		// Find Start and Goal Nodes
	    goal_node.cell = find_nearest_node(start, grid_, resolution);
	    goal_node.id = goal_node.cell.index.row_major;
	    // Set celltype to free
	    goal_node.cell.celltype = map::Free;
	    goal_node.hcost = 0.0;
	    CalculateKeys(goal_node);

	    // Find GRID cell whose coordinates most closely match the start coordinates
	    start_node.cell = find_nearest_node(goal, grid_, resolution);
	    start_node.id = start_node.cell.index.row_major;
	    start_node.hcost = heuristic(start_node, goal_node);
	    // Set celltype to free
	    start_node.cell.celltype = map::Free;
	    start_node.rhs = 0.0;
	    CalculateKeys(start_node);
	    // std::cout << "START, IDX: [" << start_node.cell.index.x << ", " << start_node.cell.index.y << "]"<< std::endl;
	    // std::cout << "GOAL, IDX: [" << goal_node.cell.index.x << ", " << goal_node.cell.index.y << "]"<< std::endl;

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

		std::cout << "Initialized!" << std::endl;
	}


	std::vector<Node> DSL::SimulateUpdate(const std::vector<Cell> & updated_grid)
	{
		std::vector<Node> updated_nodes;

		// First, make sure g(goal [start in paper]!= inf, otherwise no path)
		if (goal_node.gcost >= 1e12)
		{
			std::cout << "There is no valid path. Goal Gcost = inf." << std::endl;
			trace_path(goal_node);
			return updated_nodes;
		}

		// Change new goal node [start in D*Lite paper]
		std::priority_queue <Node, std::vector<Node>, CostComparator > pred_costs;
		// Find the predecessors of node n
		std::vector<Node> predecessors = get_neighbours(goal_node, FakeGrid);

		for (auto pred_iter = predecessors.begin(); pred_iter < predecessors.end(); pred_iter++)
    	{
    		// Find the neighbour node
    		Node predecessor;
    		predecessor = *pred_iter;
    		predecessor.id = predecessor.cell.index.row_major;

    		// Occupied or Inflated Cells
    		if (predecessor.cell.celltype == map::Occupied or\
    			predecessor.cell.celltype == map::Inflation)
    		{	    			
    			// Push to priority queue for auto sort
    			// This is not actually stored anywhere, just useful in getting min gcost for n
    			predecessor.gcost += heuristic(predecessor, goal_node) + 1e12;

    		} else
    		// Free Cells
    		{
    			// Push to priority queue for auto sort
    			// This is not actually stored anywhere, just useful in getting min gcost for n
    			predecessor.gcost += heuristic(predecessor, goal_node);
    		}

    		pred_costs.push(predecessor);
    	}

    	Node min_predecessor = pred_costs.top();


    	//Update Goal Node (start in D*L paper)
    	goal_node = FakeGrid.at(min_predecessor.id);
		GRID = updated_grid;


		updated_nodes = LPAstar::SimulateUpdate(updated_grid);

		return updated_nodes;
	}

	std::vector<Node> DSL::return_path()
	{
		std::vector<Node> p = path;
		std::reverse(p.begin(), p.end());
		return p;
	}

}	