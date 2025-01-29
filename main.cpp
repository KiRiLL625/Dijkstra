#include "Graph.h"
#include <iostream>
#include <cstdlib>
#include <ctime>

char getValidCharInput() {
    char content;
    while (true) {
        std::cin >> content;
        if (std::cin.fail() || std::cin.peek() != '\n') {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Игнорировать оставшиеся символы в потоке
            std::cout << "Invalid input. Please enter a single character: ";
        } else {
            return content; // Ввод правильный, возвращаем символ
        }
    }
}

double getValidDoubleInput() {
    double value;
    while (true) {
        std::cin >> value;
        if (std::cin.fail()) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');  // Игнорировать оставшиеся символы в потоке
            std::cout << "Invalid input. Please enter a valid number for the weight: ";
        } else {
            return value;  // Ввод правильный, возвращаем число
        }
    }
}

bool menu(Graph<char, double>& graph) {
    std::cout << "1. Add vertex\n";
    std::cout << "2. Add edge\n";
    std::cout << "3. Find shortest path and visualize\n";
    std::cout << "4. Generate random graph\n";
    std::cout << "5. Exit\n";
    int choice;
    std::cin >> choice;

    switch (choice) {
        case 1: {
            std::cout << "Enter vertex character: ";
            char content = getValidCharInput();
            graph.addVertex(content, content);
            std::cout << "Added vertex with id " << (int) content << " and character " << content << ".\n";
            return true;
        }

        case 2: {
            char from, to;
            double weight;
            std::cout << "Enter the first vertex: ";
            from = getValidCharInput();
            std::cout << "Enter the second vertex: ";
            to = getValidCharInput();
            std::cout << "Enter the weight: ";
            weight = getValidDoubleInput();
            bool directed;
            std::cout << "Is the edge directed? (0/1): ";
            std::cin >> directed;
            try {
                graph.addEdge(from, to, weight, directed);
                std::cout << "Added edge from " << from << " to " << to << " with weight " << weight << ".\n";
            }
            catch(std::invalid_argument& e){
                std::cout << "Error: " << e.what() << std::endl;
            }
            return true;
        }

        case 3: {
            char start, end;
            std::cout << "Enter the start vertex: ";
            start = getValidCharInput();
            std::cout << "Enter the end vertex: ";
            end = getValidCharInput();
            try {
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
            }
            catch(std::invalid_argument& e){
                std::cout << "Error: " << e.what() << std::endl;
            }
            return true;
        }

        case 4: {
            int numVertices;
            std::cout << "Enter the number of vertices: ";
            numVertices = static_cast<int>(getValidDoubleInput());

            // Проверка на количество вершин
            if (numVertices > 52) {
                std::cout << "Error: The number of vertices cannot exceed 52\n";
                return true;
            }

            // Генерация случайных рёбер
            bool directed;
            std::cout << "Should the edges be directed? (0/1): ";
            std::cin >> directed;

            // Генерация случайных вершин
            int vertexCount = 0;
            // Сначала заглавные буквы (A-Z)
            for (int i = 0; i < std::min(numVertices, 26); ++i) {
                char vertex = 'A' + i;  // Заглавная буква от A до Z
                graph.addVertex(vertex, vertex);
                vertexCount++;
            }

            // Если вершин больше 26, добавляем строчные буквы (a-z)
            for (int i = 26; i < numVertices; ++i) {
                char vertex = 'a' + (i - 26);  // Строчная буква от a до z
                graph.addVertex(vertex, vertex);
            }

            int numEdges = numVertices;

            srand(time(0));  // Инициализируем генератор случайных чисел
            int edgeCount = 0;
            while (edgeCount < numEdges) {
                int from = rand() % numVertices;  // Случайная вершина от 0 до numVertices-1
                int to = rand() % numVertices;    // Случайная вершина от 0 до numVertices-1

                // Убедимся, что рёбра не добавляются между одинаковыми вершинами
                if (from != to) {
                    double weight = rand() % 10 + 1;  // Случайный вес от 1 до 10
                    if (directed) {
                        graph.addEdge('A' + from, 'A' + to, weight, true);
                    } else {
                        graph.addEdge('A' + from, 'A' + to, weight, false);
                    }
                    edgeCount++;
                }
            }

            std::cout << "Random graph generated with " << numVertices << " vertices and " << numEdges << " edges.\n";
            return true;
        }

        case 5: {
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
    srand(time(0));  // Инициализация генератора случайных чисел
    while(menu(graph));
    return 0;
}
