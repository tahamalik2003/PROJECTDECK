//Taha Malik
//CSCI 335 Project 3
//Greedy Algorithim 

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <limits>
// Edge structure to represent connections between nodes
struct Edge {
    int from, to;
    double weight;

    Edge(int from, int to, double weight) : from(from), to(to), weight(weight) {}

    // Comparison operator for sorting edges
    bool operator<(const Edge& other) const {
        return weight < other.weight;
    }
};

// Function to implement the Greedy TSP algorithm
void greedyTSP(const std::string filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file.\n";
        return;
    }

    std::vector<Node> nodes;
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

    // Calculate and store all edges
    std::vector<Edge> edges;
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            double distance = nodes[i].distanceTo(nodes[j]);
            edges.emplace_back(i, j, distance);
        }
    }

    // Sort edges by weight
    std::sort(edges.begin(), edges.end());

    std::vector<int> tour; // Store the tour path
    std::vector<int> edgesAdded(nodes.size(), 0); // Track edges added to nodes

    auto startTime = std::chrono::steady_clock::now();

    // Greedy TSP logic to build the tour
    for (const auto& edge : edges) {
        if (edgesAdded[edge.from] < 2 && edgesAdded[edge.to] < 2) {
            tour.push_back(edge.from);
            tour.push_back(edge.to);
            edgesAdded[edge.from]++;
            edgesAdded[edge.to]++;
        }

        // Check for cycle
        bool cycleDetected = false;
        if (edgesAdded[edge.from] == 2 && edgesAdded[edge.to] == 2) {
            int nodeA = edge.from;
            int nodeB = edge.to;

            while (edgesAdded[nodeA] == 2 && edgesAdded[nodeB] == 2) {
                int nextA = tour[tour.size() - 2];
                int nextB = tour[tour.size() - 1];

                if (nextA != nodeB) {
                    nodeA = nextA;
                } else {
                    nodeA = nextB;
                }

                if (nextB != nodeA) {
                    nodeB = nextB;
                } else {
                    nodeB = nextA;
                }

                if (nodeA == edge.to || nodeB == edge.from) {
                    cycleDetected = true;
                    break;
                }
            }
        }

        if (!cycleDetected && tour.size() == nodes.size() * 2) {
            break;
        }
    }

    auto endTime = std::chrono::steady_clock::now();

    // Print the tour
    for (const auto& nodeId : tour) {
        std::cout << nodeId << " ";
    }
    std::cout << "\n";

    // Remaining logic to print total distance and execution time...
        // Remaining logic to print total distance and execution time...
    double totalDistance = 0.0;

    // Calculate total distance of the tour
    for (size_t i = 0; i < tour.size() - 1; i += 2) {
        int nodeA = tour[i];
        int nodeB = tour[i + 1];
        totalDistance += nodes[nodeA].distanceTo(nodes[nodeB]);
    }

    // Add distance from last node to first node to complete the tour
    int firstNode = tour[0];
    int lastNode = tour[tour.size() - 1];
    totalDistance += nodes[lastNode].distanceTo(nodes[firstNode]);

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    // Print the total distance and execution time
    std::cout << "Total Distance: " << totalDistance << "\n";
    std::cout << "Execution Time: " << duration.count() << " milliseconds\n";
}



/*
void greedyTSP(const std::string filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file.\n";
        return;
    }

    std::vector<Node> nodes;
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

    auto startTime = std::chrono::steady_clock::now();

    int current = 0;
    path.push_back(nodes[current].id);
    visited[current] = true;

    for (size_t i = 0; i < nodes.size() - 1; ++i) {
        double nearestDistance = std::numeric_limits<double>::max();
        int nearestNode = -1;

        for (size_t j = 0; j < nodes.size(); ++j) {
            if (!visited[j] && j != current) {
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

    auto endTime = std::chrono::steady_clock::now();

    for (const auto& nodeId : path) {
        std::cout << nodeId << " ";
    }
    std::cout << "\nTotal Distance: " << totalDistance << "\n";
    std::cout << "Execution Time: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count()
              << " milliseconds\n";
}
*/



/*#ifndef GREEDYTSP_HPP
#define GREEDYTSP_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <limits>
#include <algorithm>
#include <unordered_set>

// Define class representing a node in a graph, which has ID & x-y coordinates
class Node {
public:
    int id;
    double x, y;
    std::unordered_set<int> edges;

    Node(int id, double x, double y) : id(id), x(x), y(y) {}
};

// Define class representing an edge between two nodes
class Edge {
public:
    int start, end;
    double weight;

    Edge(int s, int e, double w) : start(s), end(e), weight(w) {}
};

// Function to calculate the distance between two nodes
double distance(const Node& node1, const Node& node2) {
    return std::sqrt((node1.x - node2.x) * (node1.x - node2.x) + (node1.y - node2.y) * (node1.y - node2.y));
}

// Function to find the Greedy TSP tour for a set of nodes from a given file
void greedyTSP(const std::string filename) {
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

    file.close();

    // Check if there are nodes to process
    if (nodes.empty()) {
        return;
    }

    std::vector<Edge> edges;

    // Calculate all edges
    for (size_t i = 0; i < nodes.size(); ++i) {
        for (size_t j = i + 1; j < nodes.size(); ++j) {
            edges.emplace_back(i, j, distance(nodes[i], nodes[j]));
        }
    }

    // Sort edges by weight
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        return a.weight < b.weight;
    });

    // Build the tour
    std::vector<int> tour;
    std::unordered_set<int> visited;
    tour.push_back(0); // Start at node 0
    visited.insert(0);

    for (const auto& edge : edges) {
        if (visited.find(edge.start) == visited.end() || visited.find(edge.end) == visited.end()) {
            if (nodes[edge.start].edges.size() < 2 && nodes[edge.end].edges.size() < 2) {
                tour.push_back(edge.end);
                nodes[edge.start].edges.insert(edge.end);
                nodes[edge.end].edges.insert(edge.start);
                visited.insert(edge.end);
            }
        }
    }

    // Output tour path
    for (size_t i = 0; i < tour.size(); ++i) {
        if (i > 0) std::cout << " ";
        std::cout << tour[i];
    }
    std::cout << "\n";

    // TODO: Calculate total distance and time

    // std::cout << "Total Distance: " << totalDistance << "\n";
    // std::cout << "Time in ms: " << timeInMs << "\n";
    // ... (Existing code remains unchanged)

void greedyTSP(const std::string filename) {
    // ... (Existing code remains unchanged)

    auto start = std::chrono::high_resolution_clock::now(); // Start timing

    // ... (Existing code for Greedy TSP algorithm)

    auto end = std::chrono::high_resolution_clock::now(); // End timing

    // Calculate total distance
    double totalDistance = 0.0;
    for (size_t i = 1; i < tour.size(); ++i) {
        totalDistance += distance(nodes[tour[i - 1]], nodes[tour[i]]);
    }

    // Output tour path, total distance, & execution time
    for (size_t i = 0; i < tour.size(); ++i) {
        if (i > 0) std::cout << " ";
        std::cout << tour[i];
    }
    std::cout << "\n";
    std::cout << "Total Distance: " << totalDistance << "\n";
    std::cout << "Time in ms: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
              << "\n";
}

}

#endif // GREEDYTSP_HPP
*/