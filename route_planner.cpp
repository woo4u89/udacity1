#include "route_planner.h"
#include <algorithm>
#include <vector>
#include <iostream>

using std::cout;
using std::endl;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y)
: m_Model(model) {
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
};

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    std::vector<RouteModel::Node> path_found;
    distance = 0.0f;

    while (current_node->parent != nullptr) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);

    distance *= m_Model.MetricScale();
    return path_found;
}

void RoutePlanner::AStarSearch() {
    start_node->visited = true;
    open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;

    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node->distance(*end_node) == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        } else {
            AddNeighbors(current_node);
        }
    }
}

float RoutePlanner::CalculateHValue(const RouteModel::Node *a_node) {
    return a_node->distance(*end_node);
}

RouteModel::Node* RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(),
              [](auto const &a, auto const &b) {
                  return (a->g_value + a->h_value) > (b->g_value + b->h_value);
              });
    open_list.shrink_to_fit();
    auto node = open_list.back();
    open_list.pop_back();
    return node;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *node) {
    node->FindNeighbors();
    for (RouteModel::Node *neighbor : node->neighbors) {
        neighbor->parent = node;
        float g = node->g_value;
        float d = node->distance(*neighbor);
        neighbor->g_value = g + d;
        neighbor->h_value = CalculateHValue(neighbor);
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

