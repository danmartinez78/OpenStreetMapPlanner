#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        // Add public Node variables and methods here.
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        Node* parent = nullptr;
        std::vector<Node*> neighbors{};
        void FindNeighbors();
        
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
        float distance(Node node){
          float x1 = this->x;
          float y1 = this->y;
          float x2 = node.x;
          float y2 = node.y;
          return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
          }
      private:
        // Add private Node variables and methods here.
        int index;
        RouteModel * parent_model = nullptr;
        Node * FindNeighbor(std::vector<int>);
    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);  
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.
    auto &SNodes() {return m_Nodes;}
    auto &GetNodeToRoadMap() { return node_to_road;}
    Node &FindClosestNode(float, float);


  private:
    // Add private RouteModel variables and methods here.
    std::vector<Node> m_Nodes;
    std::unordered_map< int, std::vector<const Model::Road *>> node_to_road;
    void CreateNodeToRoadHashMap();
    
};

#endif