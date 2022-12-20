#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

	// Initilize start_node and end_node
	start_node = &m_Model.FindClosestNode(start_x, start_y);
	end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	// return the distance between current_node and end_node
	return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	// Find all the neighbors for current_node
	current_node->FindNeighbors();
	
	for (auto neighbor:current_node->neighbors) {
		// Update neighbor's property
		neighbor->visited = true;
		neighbor->parent = current_node;
		neighbor->h_value = CalculateHValue(neighbor);
		neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
		
		// Add to open_list for futher search
		open_list.emplace_back(neighbor);
	}
}


RouteModel::Node *RoutePlanner::NextNode() {
	// Sort open_list base on h_value + g_value
	std::sort(open_list.begin(), open_list.end(), [](const auto &a, const auto &b) {
		return (a->h_value + a->g_value) < (b->h_value + b->g_value);
	});
	
	RouteModel::Node *lowest_node = open_list.front();
	open_list.erase(open_list.begin());
	return lowest_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
	
	// link the path together
	while (current_node->parent != nullptr) {
		path_found.push_back(*current_node);
		distance += current_node->distance(*(current_node->parent));
		current_node = current_node->parent;
	}
	path_found.push_back(*current_node);
	
	// Reverse the path so that the path start from start_node and end at end_node
	std::reverse(path_found.begin(), path_found.end());
	
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
	
	// Initialize with start_node
	start_node->visited = true;
	open_list.emplace_back(start_node);

	// A* search
	while (!open_list.empty()) {
		current_node = NextNode();
		
		// If reach end_node
		if (current_node->distance(*end_node) == 0) {
			m_Model.path = ConstructFinalPath(end_node);
			return;
		}

		// If not, keep looping until open_list is empty
		AddNeighbors(current_node);
	}
}
