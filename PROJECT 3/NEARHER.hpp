#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <limits>

// Define class representing a node in a graph, which has ID &  x-y coordinates
class Node {
public:
    int id;
    double x, y;

    Node(int id, double x, double y) : id(id), x(x), y(y) {}

    // Calculate Euclidean distance b/w this node &  another node
    double distanceTo(const Node& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

// Function to find nearest neighbor tour for a set of nodes from given file
void nearestNeighbor(const std::string filename) {
    // Store nodes from the input file.
    std::vector<Node> nodes;

    std::ifstream file(filename);
    std::string line;
    int id;
    double x, y;

    // Read node data from file & store it in 'nodes' vector
    while (getline(file, line)) {
        std::istringstream iss(line);
        if (!(iss >> id >> x >> y)) {
            continue; 
        }
        nodes.emplace_back(id, x, y);
    }

    // Check if there's nodes to process
    if (nodes.empty()) {
        return;
    }

    // Initialize data structures to track nearest neighbor tour
    std::vector<bool> visited(nodes.size(), false);

    // Store tour path
    std::vector<int> path;

    // Store total tour distance
    double totalDistance = 0.0;

    // Start measuring time
    auto start = std::chrono::high_resolution_clock::now();

    // Start from first node
    int current = 0; 

    path.push_back(nodes[current].id);
    visited[current] = true;

    // Find nearest neighbor for each unvisited node & construct tour
    for (size_t i = 1; i < nodes.size(); ++i) {
        double nearestDistance = std::numeric_limits<double>::max();
        int nearestNode = -1;

        // Find nearest unvisited node
        for (size_t j = 0; j < nodes.size(); ++j) {
            if (!visited[j]) {
                double distance = nodes[current].distanceTo(nodes[j]);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestNode = j;
                }
            }
        }

        // Handle when no nearest node is found
        if (nearestNode == -1) {
            return;
        }

        visited[nearestNode] = true;
        path.push_back(nodes[nearestNode].id);
        totalDistance += nearestDistance;
        current = nearestNode;
    }

    // Complete tour by returning to starting node
    totalDistance += nodes[current].distanceTo(nodes[0]);
    path.push_back(nodes[0].id);

    // Stop measuring time
    auto end = std::chrono::high_resolution_clock::now();

    // Output tour path, total distance, & execution time
    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) std::cout << " ";
        std::cout << path[i];
    }
    std::cout << "\n";  
    std::cout << "Total Distance: " << totalDistance << "\n";
    std::cout << "Time in ms: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
            << "\n";
}
