#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    this->m_Model = model;
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node){
    this->distance=0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node parent;

    while(current_node->parent != nullptr){
        path_found.push_back(*current_node);
        parent = *current_node->parent;
        distance += current_node->distance(parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    distance *= m_Model.MetricScale();
    return path_found;
}

void RoutePlanner::AStarSearch(){
    start_node->visited = true;
    m_open_list.push_back(start_node);
    RouteModel::Node *current_node = nullptr;
    while(m_open_list.size()>0){
        current_node = this->NextNode();
        if (current_node->distance(*end_node) == 0){
            m_Model.path = this->ConstructFinalPath(current_node);
            return;
        }
        else{
            AddNeighbors(current_node);
        }
    }
}

float RoutePlanner::CalculateHValue(RouteModel::Node *node){
    return node->distance(*end_node);
}

bool RoutePlanner::NodeComparator(RouteModel::Node *a, RouteModel::Node *b){
    // https://www.geeksforgeeks.org/sorting-a-vector-in-c/
    float a_f = a->g_value + a->h_value;
    float b_f = b->g_value + b->h_value;
    return (a_f<b_f);
}

RouteModel::Node *RoutePlanner::NextNode(){
    std::sort(m_open_list.begin(), m_open_list.end(), RoutePlanner::NodeComparator);
    RouteModel::Node *next_node = m_open_list[0];
    m_open_list.erase(m_open_list.begin());
    return next_node;   
}

void RoutePlanner::AddNeighbors(RouteModel::Node *node){
    node->FindNeighbors();
    for (RouteModel::Node *neighbor:node->neighbors){
        neighbor->parent = node;
        neighbor->g_value = node->g_value + neighbor->distance(*node);
        neighbor->h_value = this->CalculateHValue(neighbor);
        m_open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}