#include "map/grid.hpp"

namespace map
{
	using rigid2d::Vector2D;

	Cell::Cell(const Vector2D & coords_, const double & resolution_)
	{
		coords = coords_;

		double offset = euclidean_distance(resolution_, resolution_) / 2.0;

		center_coords = Vector2D(coords.x + offset, coords.y + offset);
	}

	void Grid::build_map(const double & resolution)
	{
		// Step 1. divide grid into cells based on resolution and store in class members
		xcells = arange<double>(map_min.x, map_max.x, resolution);
		ycells = arange<double>(map_min.y, map_max.y, resolution);

		// Diagonal length of one cell
		// double offset = euclidean_distance(resolution, resolution);

		// std::cout << "OFFSET: " << offset << std::endl;

		// std::cout << "Grid decomposed!" << std::endl;

		// std::cout << "# x cells: " << xcells.size() << std::endl;
		// std::cout << "# y cells: " << ycells.size() << std::endl;

		// Step 2. Populate Cell vector (cells) with world coordinates and labels in row-major-order
		for (int i = 0; i < static_cast<int>(xcells.size()); i++)
		{
			for (int j = 0; j < static_cast<int>(ycells.size()); j++)
			{
				// For each cell to be added, convert the grid index to world coords
				Cell cell(Vector2D(xcells.at(i), ycells.at(j)), resolution);

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
	}

	Index Grid::world2grid(const Cell & cell, const double & resolution)
	{
		// Get x coordinate
		int x_index = -1;
		for (int i = 0; i < static_cast<int>(xcells.size()); i++)
		{
			if (cell.coords.x >= xcells.at(i) and cell.coords.x < xcells.at(i) + resolution)
			{
				x_index = i;
			}
		}

		// Get x coordinate
		int y_index = -1;
		for (int j = 0; j < static_cast<int>(ycells.size()); j++)
		{
			if (cell.coords.y >= ycells.at(j) and cell.coords.y < ycells.at(j) + resolution)
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
		index.row_major = grid2rowmajor(x_index, y_index, static_cast<int>(xcells.size()));

		return index;
	}

	Vector2D Grid::grid2world(const int & i, const int & j, const double & resolution)
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

	std::vector<Cell> Grid::return_grid()
	{
		return cells;
	}

	void Grid::occupancy_grid(std::vector<int8_t> & map)
	{
		map.resize(cells.size());

		for(unsigned int i = 0; i < cells.size(); i++)
		{
			// Convert from row-major-order to grid coordinates
			auto index = rowmajor2grid(i, static_cast<int>(ycells.size()));
			// Convert back to rmj
			int rmj = grid2rowmajor(index.x, index.y, static_cast<int>(xcells.size()));
			index.row_major = rmj;

			cells.at(i).index = index;

			// For each cell type, assign a value to map
			if (cells.at(i).celltype == Free)
			{
				map.at(rmj) = 0;
			} else if (cells.at(i).celltype == Inflation)
			{
				map.at(rmj) = 50;
			} else if (cells.at(i).celltype == Occupied)
			{
				map.at(rmj) = 100;
			}
		}
	}

	std::vector<int> Grid::return_grid_dimensions()
	{
		std::vector<int> v;
		v.push_back(static_cast<int>(xcells.size()));
		v.push_back(static_cast<int>(ycells.size()));
		return v;
	}

	Index rowmajor2grid(const int & i, const int & numrow)
    {
        Index index;
        // NOTE: RViz uses x=col, y=row
        index.x = i / numrow;
        index.y = i % numrow;

        return index;
    }

    int grid2rowmajor(const int & i, const int & j, const int & numcol)
    {
        // NOTE: RViz uses x=col, y=row
        // index = y * num_xcols + x
        // EX: to get RMJ index 3 we do: 3 = y * num_xcols + x
        //                            => 3 = 1 + 3 + 0
        /**
        Y|(0,2)=6|(1,2)=7|(2,2)=8|
        Y|(0,1)=3|(1,1)=4|(2,1)=5|
        Y|(0,0)=0|(1,0)=1|(2,0)=2|
             X      X       X
        **/
        return j * numcol + i;
    }
}