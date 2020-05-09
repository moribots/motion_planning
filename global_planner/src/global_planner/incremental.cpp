#include "global_planner/incremental.hpp"

namespace global
{
	void LPAstar::ComputeShortestPath(const Vector2D & start, const Vector2D & goal, const map::Grid & grid_, const double & resolution)
	{

	}


	void LPAstar::UpdateCell(const Node & n, std::priority_queue <Node, std::vector<Node>, KeyComparator > & open_list, std::set<int> & open_list_v)
	{

	}


	void LPAstar::Initialize(const Vector2D & start, const Vector2D & goal, std::priority_queue <Node, std::vector<Node>, KeyComparator > & open_list,
                        	 std::set<int> & open_list_v)
	{

	}


	void LPAstar::CalculateKeys(Node & n)
	{

	}


	std::vector<Node> LPAstar::return_path()
	{
		return path;
	}

}	