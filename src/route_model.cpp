#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    auto nodes = Nodes();    
    for(int i = 0; i<nodes.size(); i++){
        Model::Node node = nodes[i];
        RouteModel::Node newNode(i,this,node);
        m_Nodes.push_back(newNode);
    }
    CreateNodeToRoadHashMap();
}

void RouteModel::CreateNodeToRoadHashMap() {
    for(const Model::Road &road : Roads()){
        if (road.type != Model::Road::Type::Footway) {
            for (int node_idx:Ways()[road.way].nodes) {
                if (node_to_road.find(node_idx)==node_to_road.end()){
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}

RouteModel::Node * RouteModel::Node::FindNeighbor(std::vector<int> node_indices){
    RouteModel::Node *closest_node = nullptr;
    Node node;
    for(int index:node_indices){
        node = this->parent_model->SNodes()[index];
        float distance_to_node = distance(node);
        if (distance_to_node>0 && !node.visited)
            if (closest_node == nullptr || distance_to_node < distance(*closest_node))
                closest_node = &parent_model->SNodes()[index];
    }
    return closest_node;
}

void RouteModel::Node::FindNeighbors(){
    for (const Model::Road * road:parent_model->node_to_road[this->index]){
        std::vector<int> node_indices = parent_model->Ways()[road->way].nodes;
        RouteModel::Node * node_ptr = FindNeighbor(node_indices);
        if (node_ptr != nullptr)
            this->neighbors.push_back(node_ptr);
    }
}

RouteModel::Node &RouteModel::FindClosestNode(float x, float y){
    RouteModel::Node input;
    input.x = x;
    input.y = y;
    float min_dist = std::numeric_limits<float>::max();
    int closest_idx;
    for (const Model::Road &road : Roads()){
        if (road.type != Model::Road::Type::Footway){
            for (int index:Ways()[road.way].nodes){
                float dist = input.distance(SNodes()[index]);
                if (dist < min_dist) {
                    closest_idx = index;
                    min_dist = dist;
                }
            }
        }
    }
    return SNodes()[closest_idx];
}