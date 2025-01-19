#include "Graph.h"
#include <iostream>

bool menu(Graph<char, double>& graph) {
    std::cout << "1. Add vertex\n";
    std::cout << "2. Add edge\n";
    std::cout << "3. Find shortest path and visualize\n";
    std::cout << "4. Exit\n";
    int choice;
    std::cin >> choice;
    switch (choice) {
        case 1: {
            char content;
            std::cout << "Enter vertex character: ";
            std::cin >> content;
            graph.addVertex(content, content);
            std::cout << "Added vertex with id " << (int) content << " and character " << content << ".\n";
            return true;
        }

        case 2: {
            char from, to;
            double weight;
            std::cout << "Enter the first vertex: ";
            std::cin >> from;
            std::cout << "Enter the second vertex: ";
            std::cin >> to;
            std::cout << "Enter the weight: ";
            std::cin >> weight;
            bool directed;
            std::cout << "Is the edge directed? (0/1): ";
            std::cin >> directed;
            graph.addEdge(from, to, weight, directed);
            std::cout << "Added edge from " << from << " to " << to << " with weight " << weight << ".\n";
            return true;
        }

        case 3: {
            char start, end;
            std::cout << "Enter the start vertex: ";
            std::cin >> start;
            std::cout << "Enter the end vertex: ";
            std::cin >> end;
            auto result = ShortestPath::dijkstra(graph, start, end);
            if (result.distance == std::numeric_limits<double>::infinity()) {
                std::cout << "No path exists between " << start << " and " << end << ".\n";
            } else {
                std::cout << "Shortest distance: " << result.distance << "\n";
                std::cout << "Path: ";
                for (size_t i = 0; i < result.path.getLength(); ++i) {
                    std::cout << (char) result.path[i];
                    if (i != result.path.getLength() - 1) std::cout << " -> ";
                }
                std::cout << "\n";
            }

            ArraySequence<Graph<char, double>::Edge> highlightedEdges;
            for (size_t i = 0; i < result.path.getLength() - 1; ++i) {
                int from = result.path[i];
                int to = result.path[i + 1];
                const auto *edges = graph.getOutgoingEdges(from);
                if (edges != nullptr) {
                    for (const auto &edge: *edges) {
                        if (edge.to == to) {
                            highlightedEdges.append(edge);
                            break;
                        }
                    }
                }
            }
            graph.visualize("graph.dot", highlightedEdges);
            std::system("dot -Tpng graph.dot -o graph.png");
            return true;
        }

        case 4: {
            return false;
        }

        default: {
            std::cout << "Invalid choice.\n";
            return true;
        }
    }
}

int main() {
    Graph<char, double> graph;
    int vertexCount = 0;
    while(menu(graph));
    return 0;
}