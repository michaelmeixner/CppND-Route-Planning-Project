#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    start_x /= 100;
    start_y /= 100;
    end_x /= 100;
    end_y /= 100;
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
  	for(auto& node : current_node->neighbors) {
  	  node->parent = current_node;
      node->h_value = CalculateHValue(node);
      node->g_value = node->distance(*current_node) + current_node->g_value;
      open_list.emplace_back(node);
      node->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {
    sort(open_list.begin(), open_list.end(), [] (const RouteModel::Node* a, const RouteModel::Node* b) {
      return a->g_value + a->h_value > b->g_value + b->h_value;
    });
  	RouteModel::Node *node = open_list.back();
    open_list.pop_back();
  	return node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while(current_node != start_node) {
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.emplace_back(*start_node);
    distance *= m_Model.MetricScale();
  	reverse(path_found.begin(), path_found.end());
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
  	start_node->visited = true;
  	open_list.emplace_back(start_node);
    while(!open_list.empty()) {
    	current_node = NextNode();
      	if(current_node == end_node) { 
        	m_Model.path = ConstructFinalPath(end_node); 
          	return;
        }
      	AddNeighbors(current_node);
    }
}