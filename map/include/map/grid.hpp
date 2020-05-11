#ifndef GRID_INCLUDE_GUARD_HPP
#define GRID_INCLUDE_GUARD_HPP
/// \file
/// \brief GRID Library to build a Probabilistic Roadmap.
#include <map/map.hpp>
#include <map/prm.hpp> // to use PRM::too_close method

namespace map
{
    using rigid2d::Vector2D;

    // \brief stores indeces to acces elements in row-major order or grid-wise
    struct Index{
        int row_major = -1;
        int x = -1;
        int y = -1;

    };

    enum CellType {Occupied, Inflation, Free};

    // \brief struct to store Cell paramters such as coordinates, center coordinates,
    // whether a cell has been visited, and the cell type (Start, Goal, Obstacle, Standard)
    struct Cell{

        // \brief Cell constructor with input coordinates
        Cell(const Vector2D & coords_, const double & resolution_);

        // Corner coordinates
        Vector2D coords;
        // Centre coordinates
        Vector2D center_coords;
        bool visited = false;
        // Default Cell Type
        CellType celltype = Free;
        // Cell value
        double value = 0.0;
        // Cell index in grid (also RMJ)
        Index index;
        double resolution;
        // Used for simulated grid update
        bool newView = false;
    };

    /// \brief stores Obstacle(s) to construct basic Grid. Inherits from Map in map.hpp.
    class Grid : public Map
    {
        // Inherits Constructors
        using Map::Map;

    public:

        // \brief Constructs a Grid Map.
        // \param resolution: determines the grid cell size
        void build_map(const double & resolution);

        // \brief converts world coordinates to grid coordinates and returns result
        // \param cell: a grid cell
        // \returns the grid cell's world coordinates
        Index world2grid(const Cell & cell) const;

        // \brief converts grid coordinates to world coordinates and returns result
        // \param i: x-coordinate of grid cell
        // \param j: y-coordinate of grid cell
        // \param resolution: determines the grid cell size
        // \returns the grid cell's grid coordinates
        Vector2D grid2world(const int & i, const int & j, const double & resolution) const;

        // \brief Return Grid in row-major order
        // \returns vector containing grid cells as Cell
        std::vector<Cell> return_grid() const;

        // \brief populates Occupancy Grid with values for visualization
        // \param map: the Occupancy Grid map to populate
        void occupancy_grid(std::vector<int8_t> & map) const;

        // \brief returns the width and height of the grid in cells
        // \returns int vector containing width and height respectively.
        std::vector<int> return_grid_dimensions() const;

        // \brief gets the neighbours of a given cell according to a visibility bound
        // \param current_cell: the cell from which we simulate the update
        // \param visibility: size of bounding box used for update (can vary each iteration if desired). DEFAULT is 1 for 3x3 eval.
        // \returns: vector of neighbour Cells
        std::vector<Cell> get_neighbours(const Cell & cc, const std::vector<Cell> & map, const int & visibility=1);

        // \brief Increment fake copy of grid for simulated updates
        // \param current_cell: the cell from which we simulate the update
        // \param visibility: size of bounding box used for update (can vary each iteration if desired)
        void update_grid(const Cell & cc, const int & visibility);

        // \brief Return FAKE (simulated increment) Grid in row-major order
        // \returns vector containing grid cells as Cell
        std::vector<Cell> return_fake_grid() const;

        // \brief populates Occupancy Grid with FAKE values for visualization
        // \param map: the Occupancy Grid map to populate
        void fake_occupancy_grid(std::vector<int8_t> & map) const;

        // \brief Checks whether a potential Vertex lies on an Obstacle. 'map::PRM::sample_configurations' calls this function.
        // \param q: the Vertex being examined
        friend bool not_inside(const Vertex & q, const std::vector<Obstacle> & obstacles, const double & inflate_robot);

        // \brief Checks if a Vertex is too close to an Edge.
        // \param E1: the first Vertex forming an edge
        // \param E2: the second Vertex forming an edge
        // \param P0: the Vertex whose closeness is being examined.
        // \param inflate_robot: approximate robot radius used for collision checking.
        friend bool too_close(const Vertex & E1, const Vertex & E2, const Vertex & P0, const double & inflate_robot);

    private:
        std::vector<Cell> cells;
        std::vector<Cell> fake_grid;
        std::vector<double> xcells;
        std::vector<double> ycells;
    };

    // \brief numpy arange in C++, can take any type T (int,double,etc)
    template<typename T>
    std::vector<T> arange(const T & start, const T & stop, const T & step = 1)
    {
        std::vector<T> values;
        for (T value = start; value < stop; value += step)
            values.push_back(value);
        return values;
    }

    // \brief convert from row-major-order coordinates to grid coordinates
    // \param rmj: row-major-order coordinate
    // \param numrow: number of rows in grid
    // \returns Index containing grid coordintates
    Index rowmajor2grid(const int & rmj, const int & numrow);

    // \brief convert from grid coordinates to row-major-order coordinates
    // \param x: x-coordinate of grid cell
    // \param y: y-coordinate of grid cell
    // \param numcol: number of columns in grid
    // \returns row-major-order index
    int grid2rowmajor(const int & x, const int & y, const int & numrow);
}

#endif
