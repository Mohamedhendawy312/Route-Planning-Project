# A* Route Planning on OpenStreetMap

A C++ implementation of the A* search algorithm to find the shortest path between two points on real OpenStreetMap data.

<img src="map.png" width="600" height="450" />

## What It Does

Given a start and end location (x,y coordinates), this program:
1. Loads OpenStreetMap XML data
2. Finds the closest road nodes to your coordinates
3. Runs A* search to find the optimal path
4. Displays the route visually on the map
5. Reports the total distance in meters

## How A* Works

A* is a graph search algorithm that finds the shortest path by combining:
- **g(n)**: Actual cost from start to current node
- **h(n)**: Heuristic estimate to goal (Euclidean distance)
- **f(n) = g(n) + h(n)**: Total estimated cost

It always explores the most promising node first (lowest f-value), guaranteeing the shortest path while being more efficient than exploring blindly.

## Project Structure

```
├── src/
│   ├── main.cpp           # Entry point, user input, rendering
│   ├── route_planner.cpp  # A* algorithm implementation
│   ├── route_planner.h
│   ├── route_model.cpp    # Graph data structure for OSM
│   ├── route_model.h
│   ├── model.cpp          # OSM XML parsing (provided)
│   ├── model.h
│   ├── render.cpp         # Map visualization (provided)
│   └── render.h
├── test/
│   └── utest_rp_a_star_search.cpp  # Unit tests
├── map.osm                # Sample map data
└── map.png                # Example output
```

## Dependencies

- cmake >= 3.11.3
- make >= 4.1
- gcc/g++ >= 7.4.0
- [IO2D](https://github.com/cpp-io2d/P0267_RefImpl/blob/master/BUILDING.md) - Graphics library for rendering

## Building

```bash
# Clone with submodules
git clone https://github.com/Mohamedhendawy312/Route-Planning-Project.git --recurse-submodules
cd Route-Planning-Project

# Build
mkdir build && cd build
cmake ..
make
```

## Usage

```bash
# Run with default map
./OSM_A_star_search

# Run with custom map
./OSM_A_star_search -f ../your_map.osm
```

You'll be prompted to enter start and end coordinates (0-99 scale).

## Testing

```bash
cd build
./test
```

Runs 4 unit tests covering:
- Heuristic calculation
- Neighbor expansion
- Path construction
- Full A* search

## Author

Mohamed Hendawy
