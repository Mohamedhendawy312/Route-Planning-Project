#include "route_planner.h"
#include <algorithm>

/**
 * @brief Constructs a RoutePlanner with start and end coordinates
 * @param model Reference to the RouteModel containing map data
 * @param start_x Start X coordinate (0-100 scale)
 * @param start_y Start Y coordinate (0-100 scale)
 * @param end_x End X coordinate (0-100 scale)
 * @param end_y End Y coordinate (0-100 scale)
 */
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model)
{
    // Convert inputs to percentage (0-1 scale)
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


/**
 * @brief Calculates the heuristic value (distance to end node)
 * @param node Pointer to the node to evaluate
 * @return Euclidean distance from node to end_node
 */
float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*end_node);
}


/**
 * @brief Expands current node by adding unvisited neighbors to open list
 * @param current_node Pointer to the node being expanded
 */
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    // Populate current node's neighbors vector
    current_node->FindNeighbors();

    // Process each neighbor: set parent, calculate g and h values
    for (auto neighbor : current_node->neighbors)
    {
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);
        open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
}


/**
 * @brief Sorts open list and returns the node with lowest f-value (g + h)
 * @return Pointer to the most promising node for exploration
 */
RouteModel::Node *RoutePlanner::NextNode()
{
    // Sort by f-value (g + h) in ascending order
    std::sort(open_list.begin(), open_list.end(), [](const auto &a, const auto &b)
    {
        return (a->h_value + a->g_value) < (b->h_value + b->g_value);
    });

    // Get the node with lowest value and remove it from the list
    RouteModel::Node* lowest_node = open_list.front();
    open_list.erase(open_list.begin());
    return lowest_node;
}


/**
 * @brief Reconstructs the path from end node back to start
 * @param current_node The end node (destination)
 * @return Vector of nodes representing the path from start to end
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Walk backwards through parents until we reach start
    while (current_node != start_node)
    {
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }

    // Add the start node
    path_found.push_back(*current_node);

    // Reverse to get correct order (start -> end)
    std::reverse(path_found.begin(), path_found.end());

    // Convert to real-world distance
    distance *= m_Model.MetricScale();

    return path_found;
}


/**
 * @brief Performs A* search algorithm to find shortest path
 * 
 * Explores nodes starting from start_node, always expanding the most
 * promising node (lowest g + h value) until reaching end_node.
 */
void RoutePlanner::AStarSearch()
{
    // Initialize with start node
    start_node->visited = true;
    open_list.emplace_back(start_node);

    // Main search loop
    while (!open_list.empty())
    {
        // Get the most promising node
        RouteModel::Node* current_node = NextNode();

        // Check if we've reached the goal
        if (current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(end_node);
            return;
        }

        // Expand this node's neighbors
        AddNeighbors(current_node);
    }
}