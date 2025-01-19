#include <iostream>
#include "ArraySequence.h"
#include "Dictionary.h"
#include "PriorityQueue.h"
#include <limits>
#include <stack>
#include <algorithm>
#include <fstream>

template<typename VertexContent, typename EdgeContent>
class Graph {
public:
    struct Vertex {
        int id; // идентификатор вершины
        VertexContent content; // содержимое вершины

        Vertex(): id(-1), content(VertexContent()) {}

        Vertex(int id, const VertexContent& content) : id(id), content(content) {}
    };

    struct Edge {
        int id; // идентификатор дуги (опционально)
        int from; // идентификатор начальной вершины
        int to; // идентификатор конечной вершины
        EdgeContent content; // содержимое дуги

        Edge(): id(-1), from(-1), to(-1), content(EdgeContent()) {}

        Edge(int from, int to, const EdgeContent& content, int id = -1)
            : id(id), from(from), to(to), content(content) {}
    };

    void addVertex(int id, const VertexContent& content) {
        vertices[id] = Vertex(id, content);
    }

    void addEdge(int from, int to, const EdgeContent& content, bool directed = false, int id = -1) {
        if (!edges.contains_key(from)) {
            edges[from] = new ArraySequence<Edge>();
        }
        edges[from]->append(Edge(from, to, content, id));
        if (!directed) {
            if (!edges.contains_key(to)) {
                edges[to] = new ArraySequence<Edge>();
            }
            edges[to]->append(Edge(to, from, content, id));
        }
    }

    const Dictionary<int, Vertex>& getVertices() const {
        return vertices;
    }

    const ArraySequence<Edge>* getOutgoingEdges(int vertexId) const {
        if (edges.contains_key(vertexId)) {
            return edges.get(vertexId);
        }
        return nullptr;
    }


    const Dictionary<int, ArraySequence<Edge>*>& getEdges() const {
        return edges;
    }

    void visualize(const std::string& filename) const {
        std::ofstream outFile(filename);

        outFile << "digraph G {\n"; // directed graph

        // Создаем вершины
        for (const auto& pair : vertices) {
            if(pair.value.id != -1) {
                outFile << "  " << pair.value.id << " [label=\"" << pair.value.content << "\"];\n";
            }
        }

        // Создаем рёбра
        for (const auto& pair : edges) {
            if (pair.value == nullptr) {
                //std::cerr << "Ошибка: указатель на рёбер для вершины " << pair.key << " равен nullptr.\n";
                continue;
            }
            for (const auto& edge : *pair.value) {
                outFile << "  " << edge.from << " -> " << edge.to << " [label=\"" << edge.content << "\"];\n";
            }
        }

        outFile << "}\n";
        outFile.close();
    }

    ~Graph() {
        for (const auto& pair : edges) {
            delete pair.value;
        }
    }

private:
    Dictionary<int, Vertex> vertices; // словарь вершин
    Dictionary<int, ArraySequence<Edge>*> edges; // словарь рёбер, принадлежащих вершине
};

class ShortestPath {
public:
    struct Result {
        double distance; // расстояние
        ArraySequence<int> path; // последовательность вершин пути
    };

    // Алгоритм Дейкстры
    template<typename VertexContent, typename EdgeContent>
    static Result dijkstra(const Graph<VertexContent, EdgeContent>& graph, int start, int end) {
        Dictionary<int, double> distances; // словарь для хранения расстояний от начальной вершины до остальных
        Dictionary<int, int> previous; // словарь для хранения предыдущих вершин на пути к вершине

        for (const auto& pair : graph.getEdges()) { // проходим по всем вершинам графа
            distances[pair.key] = std::numeric_limits<double>::infinity(); // расстояние до вершины равно бесконечности
            previous[pair.key] = -1; // предыдущей вершины нет
        }
        distances[start] = 0; // расстояние от начальной вершины до неё самой равно 0

        using P = std::pair<double, int>; // {distance, vertex}

        PriorityQueue<P> pq; // приоритетная очередь для хранения вершин
        pq.append({0, start}); // добавляем начальную вершину

        while (!pq.empty()) { // пока очередь не пуста
            auto [currentDistance, currentVertex] = pq.getFirst(); // получаем вершину с минимальным расстоянием
            pq.pop(); // удаляем её из очереди

            if (currentDistance > distances[currentVertex]) { // если расстояние до вершины больше, чем текущее расстояние
                continue; // то пропускаем эту вершину
            }

            // Проходим по всем смежным вершинам
            for (const auto& edge : *graph.getOutgoingEdges(currentVertex)) {
                double newDist = currentDistance + edge.content; // новое расстояние - это текущее расстояние плюс вес ребра
                if (newDist < distances[edge.to]) { // если новое расстояние меньше, чем текущее расстояние
                    distances[edge.to] = newDist; // то обновляем расстояние
                    previous[edge.to] = currentVertex; // и запоминаем предыдущую вершину
                    pq.append({newDist, edge.to}); // добавляем вершину в очередь
                }
            }
        }

        // Восстановление пути
        ArraySequence<int> path; // последовательность вершин пути
        for (int at = end; at != -1; at = previous[at]) { // проходим по всем предыдущим вершинам
            path.append(at); // добавляем вершину в путь
        }
        std::reverse(path.begin(), path.end()); // переворачиваем путь

        if (path.getLength() == 1 && path[0] != start) { // если путь состоит из одной вершины и это не начальная вершина
            // Если конечная вершина недостижима
            return {std::numeric_limits<double>::infinity(), {}};
        }

        return {distances[end], path}; // возвращаем расстояние и путь
    }
};

int main() {
    Graph<std::string, double> graph;

    // Добавление вершин
    graph.addVertex(1, "A");
    graph.addVertex(2, "B");
    graph.addVertex(3, "C");
    graph.addVertex(4, "D");

    // Добавление рёбер
    graph.addEdge(1, 2, 1.5);
    graph.addEdge(2, 3, 2.0);
    graph.addEdge(3, 4, 1.0);
    graph.addEdge(4, 1, 3.5);
    graph.addEdge(3, 1, 3.0, true);

    // Указание начальной и конечной вершин
    int start = 1;
    int end = 3;

    // Поиск кратчайшего пути
    auto result = ShortestPath::dijkstra(graph, start, end);

    // Вывод результатов
    if (result.distance == std::numeric_limits<double>::infinity()) {
        std::cout << "No path exists between " << start << " and " << end << ".\n";
    } else {
        std::cout << "Shortest distance: " << result.distance << "\n";
        std::cout << "Path: ";
        for (size_t i = 0; i < result.path.getLength(); ++i) {
            std::cout << result.path[i];
            if (i != result.path.getLength() - 1) std::cout << " -> ";
        }
        std::cout << "\n";
    }

    // Визуализация графа
    graph.visualize("graph.dot");
    return 0;
}