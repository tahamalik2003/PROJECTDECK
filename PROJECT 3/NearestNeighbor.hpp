//Taha Malik
//CSCI 335 Project 3
//Nearest Neighbor ALgorithim 

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <limits>

class Node {
public:
    int id;
    double x, y;

    // Node constructor
    Node(int id, double x, double y) : id(id), x(x), y(y) {}

    // Method to calculate distance between two nodes
    double distanceTo(const Node& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

// Function to find the nearest neighbor path for a set of nodes
void nearestNeighbor(const std::string filename) {
    // Open the file
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file.\n"; // Display error if file opening fails
        return;
    }

    // Vector to store nodes read from the file
    std::vector<Node> nodes;
    std::string line;
    int id;
    double x, y;

    // Read nodes data from the file
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (!(iss >> id >> x >> y)) {
            continue;
        }
        // Create Node objects and add them to the vector
        nodes.emplace_back(id, x, y);
    }

    // Check if nodes were loaded
    if (nodes.empty()) {
        std::cout << "No nodes to process.\n";
        return;
    }

    // Initialization for the algorithm
    std::vector<bool> visited(nodes.size(), false); // Tracks visited nodes
    std::vector<int> path; // Stores the path of nodes
    double totalDistance = 0.0; // Accumulates the total distance of the path

    auto startTime = std::chrono::steady_clock::now(); // Start time for execution measurement

    int current = 0; // Starting node index
    path.push_back(nodes[current].id); // Add the starting node to the path
    visited[current] = true; // Mark the starting node as visited

    // Find the nearest neighbor for each node
    while (path.size() < nodes.size()) {
        double nearestDistance = std::numeric_limits<double>::max(); // Initialize with maximum possible value
        int nearestNode = -1; // Initialize with invalid index

        // Loop through all nodes to find the nearest unvisited node
        for (size_t j = 0; j < nodes.size(); ++j) {
            if (!visited[j]) {
                double distance = nodes[current].distanceTo(nodes[j]);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestNode = j;
                }
            }
        }

        // Check if a nearest unvisited node was found
        if (nearestNode == -1) {
            std::cerr << "Error: Unable to find nearest node.\n";
            return;
        }

        // Mark the nearest node as visited, update path and distance information
        visited[nearestNode] = true;
        path.push_back(nodes[nearestNode].id);
        totalDistance += nearestDistance;
        current = nearestNode; // Update current node for the next iteration
    }

    // Complete the cycle by returning to the starting node
    totalDistance += nodes[current].distanceTo(nodes[0]);
    path.push_back(nodes[0].id);

    auto endTime = std::chrono::steady_clock::now(); // End time for execution measurement

    // Display the found path, total distance, and execution time
    for (const auto& nodeId : path) {
        std::cout << nodeId << " ";
    }
    std::cout << "\nTotal Distance: " << totalDistance << "\n";
    std::cout << "Execution Time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count()
              << " milliseconds\n";
}





/*#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <limits>

class Node {
public:
    int id;
    double x, y;

    Node(int id, double x, double y) : id(id), x(x), y(y) {}

    double distanceTo(const Node& other) const {
        return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y));
    }
};

void nearestNeighbor(const std::string filename) {
    std::vector<Node> nodes;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file.\n";
        return;
    }

    std::string line;
    int id;
    double x, y;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        if (!(iss >> id >> x >> y)) {
            continue;
        }
        nodes.emplace_back(id, x, y);
    }

    if (nodes.empty()) {
        std::cout << "No nodes to process.\n";
        return;
    }

    std::vector<bool> visited(nodes.size(), false);
    std::vector<int> path;
    double totalDistance = 0.0;

    auto start = std::chrono::steady_clock::now();

    int current = 0;
    path.push_back(nodes[current].id);
    visited[current] = true;

    for (size_t i = 1; i < nodes.size(); ++i) {
        double nearestDistance = std::numeric_limits<double>::max();
        int nearestNode = -1;

        for (size_t j = 0; j < nodes.size(); ++j) {
            if (!visited[j]) {
                double distance = nodes[current].distanceTo(nodes[j]);
                if (distance < nearestDistance) {
                    nearestDistance = distance;
                    nearestNode = j;
                }
            }
        }

        if (nearestNode == -1) {
            std::cerr << "Error: Unable to find nearest node.\n";
            return;
        }

        visited[nearestNode] = true;
        path.push_back(nodes[nearestNode].id);
        totalDistance += nearestDistance;
        current = nearestNode;
    }

    totalDistance += nodes[current].distanceTo(nodes[0]);
    path.push_back(nodes[0].id);

    auto end = std::chrono::steady_clock::now();

    for (size_t i = 0; i < path.size(); ++i) {
        if (i > 0) std::cout << " ";
        std::cout << path[i];
    }
    std::cout << "\nTotal Distance: " << totalDistance << "\n";
    std::cout << "Time elapsed: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << " milliseconds\n";
}

*/












