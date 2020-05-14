#include "map/grid.hpp"

namespace map
{
	using rigid2d::Vector2D;

	Cell::Cell(const Vector2D & coords_, const double & resolution_)
	{
		coords = coords_;

		double offset = resolution_ / 2.0;

		center_coords = Vector2D(coords.x + offset, coords.y + offset);
		resolution = resolution_;
	}

	void Grid::build_map(const double & resolution)
	{
		// Step 1. divide grid into cells based on resolution and store in class members
		xcells = arange<double>(map_min.x, map_max.x, resolution);
		ycells = arange<double>(map_min.y, map_max.y, resolution);

		// Step 2. Populate Cell vector (cells) with world coordinates and labels in row-major-order
		for (int i = 0; i < static_cast<int>(ycells.size()); i++)
		{
			for (int j = 0; j < static_cast<int>(xcells.size()); j++)
			{
				// std::cout << "[" << j << ", " << i << "]" << std::endl;
				// For each cell to be added, convert the grid index to world coords
				Cell cell(Vector2D(xcells.at(j), ycells.at(i)), resolution);

				// Perform inside-obstacle check for labeling Obstacle
				if (!not_inside(Vertex(cell.center_coords), obstacles, 0.0))
				{
					// std::cout << "OCCUPIED: [" << cell.center_coords.x << ", " << cell.center_coords.y << "]" << std::endl;
					cell.celltype = Occupied;
				} else if (!not_inside(Vertex(cell.center_coords), obstacles, inflate_robot))
				// Perform close-to-obstacle check for labeling Inflation
				{
					cell.celltype = Inflation;
					// std::cout << "INFLATED: [" << cell.center_coords.x << ", " << cell.center_coords.y << "]" << std::endl;
				}
				cells.push_back(cell);
			}
		}

		// Populate more cell information such as row-major order and grid index
		for(unsigned int i = 0; i < cells.size(); i++)
		{
			// Convert from row-major-order to grid coordinates
			auto index = rowmajor2grid(i, static_cast<int>(xcells.size()));
			// Convert back to rmj
			int rmj = grid2rowmajor(index.x, index.y, static_cast<int>(xcells.size()));
			index.row_major = rmj;

			if (!(rmj == static_cast<int>(i)))
			{
				throw std::runtime_error("Row-Major Order Conversion Failed!\
										 \n  where(): Grid::build_map(const double & resolution)");
			}

			cells.at(i).index = index;

			// Set fake_grid to same cells but all free
			Cell fake_cell = cells.at(i);
			fake_cell.celltype = Free;
			fake_grid.push_back(fake_cell);
		}
	}

	Index Grid::world2grid(const Cell & cell) const
	{
		// Get x coordinate
		int x_index = -1;
		for (int i = 0; i < static_cast<int>(xcells.size()); i++)
		{
			if (cell.coords.x >= xcells.at(i) and cell.coords.x < xcells.at(i) + cell.resolution)
			{
				x_index = i;
			}
		}

		// Get x coordinate
		int y_index = -1;
		for (int j = 0; j < static_cast<int>(ycells.size()); j++)
		{
			if (cell.coords.y >= ycells.at(j) and cell.coords.y < ycells.at(j) + cell.resolution)
			{
				y_index = j;
			}
		}

		if (x_index == -1 or y_index == -1)
		{
			throw std::runtime_error("Could not convert from world to grid coordinates!");
		}

		Index index;
		index.x = x_index;
		index.y = y_index;
		index.row_major = grid2rowmajor(index.x, index.y, static_cast<int>(xcells.size()));

		return index;
	}

	Vector2D Grid::grid2world(const int & i, const int & j, const double & resolution) const
	{
		if (!(i >= 0 and i <= static_cast<int>(xcells.size()) - 1))
		{
			throw std::invalid_argument("cell's x coordinate out of bounds!");
		} else if (!(j >= 0 and j <= static_cast<int>(ycells.size()) - 1))
		{
			throw std::invalid_argument("cell's y coordinate out of bounds!");
		}
		Vector2D coord;
		coord.x = i * resolution + map_min.x;
		coord.y = j * resolution + map_min.y;
		return coord;
	}

	std::vector<Cell> Grid::return_grid() const
	{
		return cells;
	}

	void Grid::occupancy_grid(std::vector<int8_t> & map) const
	{
		map.resize(cells.size());

		for(unsigned int i = 0; i < cells.size(); i++)
		{
			// For each cell type, assign a value to map
			if (cells.at(i).celltype == Free)
			{
				map.at(i) = 0;
			} else if (cells.at(i).celltype == Inflation)
			{
				map.at(i) = 50;
			} else if (cells.at(i).celltype == Occupied)
			{
				map.at(i) = 100;
			}
		}
	}

	std::vector<int> Grid::return_grid_dimensions() const
	{
		std::vector<int> v;
		v.push_back(static_cast<int>(xcells.size()));
		v.push_back(static_cast<int>(ycells.size()));
		return v;
	}


	void Grid::update_grid(const Cell & cc, const int & visibility)
	{

		std::vector<Cell> cells_to_update = get_neighbours(cc, cells, visibility);

		for (auto iter = cells_to_update.begin(); iter < cells_to_update.end(); iter++)
		{
			if (fake_grid.at(iter->index.row_major).celltype != iter->celltype)
			{
				// std::cout << "NEW OCCUPANCY: " << iter->index.row_major << std::endl;
				fake_grid.at(iter->index.row_major).celltype = iter->celltype;
				fake_grid.at(iter->index.row_major).newView = true;
			} else
			{
				fake_grid.at(iter->index.row_major).newView = false;
			}
		}
	}


	std::vector<Cell> Grid::return_fake_grid() const
	{
		return fake_grid;
	}


	void Grid::fake_occupancy_grid(std::vector<int8_t> & map) const
	{
		map.resize(fake_grid.size());

		for(unsigned int i = 0; i < fake_grid.size(); i++)
		{
			// For each cell type, assign a value to map
			if (fake_grid.at(i).celltype == Free)
			{
				map.at(i) = 0;
			} else if (fake_grid.at(i).celltype == Inflation)
			{
				map.at(i) = 50;
			} else if (fake_grid.at(i).celltype == Occupied)
			{
				map.at(i) = 100;
			}
		}
	}

	std::vector<Cell> Grid::get_neighbours(const Cell & cc, const std::vector<Cell> & map, const int & visibility)
	{
		int lower_bound = - visibility;
		int upper_bound = - lower_bound;
		int x_max = map.back().index.x;
		int y_max = map.back().index.y; 
		std::vector<Cell> neighbours;

		// Evaluate about block. Default is 1 -> 3x3 for 8-connectivity
		for (int x = lower_bound; x <= upper_bound; x++)
		{
			for (int y = lower_bound; y <= upper_bound; y++)
			{
				// Skip x,y = (0,0) since that's the current node
				if (x == 0 and y == 0)
				{
					continue;
				} else
				{
					int check_x = cc.index.x + x;
					int check_y = cc.index.y + y;

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



	Index rowmajor2grid(const int & rmj, const int & numcol)
    {
        Index index;
        // NOTE: RViz uses x=col, y=row
        index.y = rmj / numcol;
        index.x = rmj - (index.y * numcol);

        return index;
    }

    int grid2rowmajor(const int & x, const int & y, const int & numcol)
    {
        // NOTE: RViz uses x=col, y=row
        // So numcol is the number of x squares in each y row so it's x.size()
        // index = y * num_row + x
        // EX: to get RMJ index 3 we do: 3 = y * num_col + x
        //                            => 3 = 1 * 3 + 0
        /**
        Y|(0,3)=9|(1,3)=10|(2,3)=11|
        Y|(0,2)=6|(1,2)= 7|(2,2)= 8|
        Y|(0,1)=3|(1,1)= 4|(2,1)= 5|
        Y|(0,0)=0|(1,0)= 1|(2,0)= 2|
             X      X       X
        **/
        return x + y * numcol;
    }
}