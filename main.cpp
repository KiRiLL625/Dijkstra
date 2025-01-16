#include <iostream>
#include "ArraySequence.h"
#include "Dictionary.h"
#include "PriorityQueue.h"
#include <limits>
#include <stack>
#include <algorithm>

class Graph {
public:
    struct Vertex {
        int id; //идентификатор вершины
        double x, y; //координаты вершины
    };

    struct Edge {
        int from; //идентификатор начальной вершины
        int to; //идентификатор конечной вершины
        double weight; //вес ребра

        Edge() : from(0), to(0), weight(0.0) {} //конструктор по умолчанию

        // Конструктор с параметрами
        Edge(int from, int to, double weight) : from(from), to(to), weight(weight) {}

        // Конструктор копирования
        Edge(const Edge& other) : from(other.from), to(other.to), weight(other.weight) {}

        // Оператор копирующего присваивания
        Edge& operator=(const Edge& other) {
            if (this != &other) {
                from = other.from;
                to = other.to;
                weight = other.weight;
            }
            return *this;
        }

        // Оператор вывода в поток
        friend std::ostream& operator<<(std::ostream& os, const Edge& edge) {
            os << "Edge(from: " << edge.from << ", to: " << edge.to << ", weight: " << edge.weight << ")";
            return os;
        }
    };

    void addVertex(int id, double x, double y) { //добавление вершины
        vertices[id] = {id, x, y};
    }

    // Добавление ребра
    void addEdge(int from, int to, double weight, bool directed = false) {
        if (!edges.contains_key(from)) { //если ребра нет, то создаем его
            edges[from] = new ArraySequence<Edge>();
        }
        edges[from]->append(Edge(from, to, weight)); //добавляем ребро
        if (!directed) { //если граф неориентированный
            if (!edges.contains_key(to)) {
                edges[to] = new ArraySequence<Edge>();
            }
            edges[to]->append(Edge(to, from, weight)); //то добавляем ребро в обратную сторону
        }
    }

    const Dictionary<int, ArraySequence<Edge>*>& getEdges() const { //получаем последовательность рёбер
        return edges;
    }

    ~Graph() { //деструктор
        for (const auto& pair : edges) {
            delete pair.value;
        }
    }

private:
    Dictionary<int, Vertex> vertices; //словарь вершин
    Dictionary<int, ArraySequence<Edge>*> edges; //словарь рёбер, принадлежащих вершине
};

class ShortestPath {
public:
    struct Result {
        double distance; //расстояние
        ArraySequence<int> path; //последовательность вершин пути
    };

    // Алгоритм Дейкстры
    static Result dijkstra(const Graph& graph, int start, int end) {
        Dictionary<int, double> distances; //словарь для хранения расстояний от начальной вершины до остальных
        Dictionary<int, int> previous; //словарь для хранения предыдущих вершин на пути к вершине

        for (const auto& pair : graph.getEdges()) { //проходим по всем вершинам графа
            distances[pair.key] = std::numeric_limits<double>::infinity(); //расстояние до вершины равно бесконечности
            previous[pair.key] = -1; //предыдущей вершины нет
        }
        distances[start] = 0; //расстояние от начальной вершины до неё самой равно 0

        using P = std::pair<double, int>; // {distance, vertex}

        PriorityQueue<P> pq; //приоритетная очередь для хранения вершин
        pq.append({0, start}); //добавляем начальную вершину

        while (!pq.empty()) { //пока очередь не пуста

            auto [currentDistance, currentVertex] = pq.getFirst(); //получаем вершину с минимальным расстоянием
            pq.pop(); //удаляем её из очереди

            if (currentDistance > distances[currentVertex]) { //если расстояние до вершины больше, чем текущее расстояние
                continue; //то пропускаем эту вершину
            }

            // Проходим по всем смежным вершинам
            for (const auto& edge : *graph.getEdges().get(currentVertex)) {
                double newDist = currentDistance + edge.weight; //новое расстояние - это текущее расстояние плюс вес ребра
                if (newDist < distances[edge.to]) { //если новое расстояние меньше, чем текущее расстояние
                    distances[edge.to] = newDist; //то обновляем расстояние
                    previous[edge.to] = currentVertex; //и запоминаем предыдущую вершину
                    pq.append({newDist, edge.to}); //добавляем вершину в очередь
                }
            }
        }

        // Восстановление пути
        ArraySequence<int> path; //последовательность вершин пути
        for (int at = end; at != -1; at = previous[at]) { //проходим по всем предыдущим вершинам
            path.append(at); //добавляем вершину в путь
        }
        std::reverse(path.begin(), path.end()); //переворачиваем путь

        if (path.getLength() == 1 && path[0] != start) { //если путь состоит из одной вершины и это не начальная вершина
            // Если конечная вершина недостижима
            return {std::numeric_limits<double>::infinity(), {}};
        }

        return {distances[end], path}; //возвращаем расстояние и путь
    }
};

int main() {
    Graph graph;

    // Добавление вершин
    graph.addVertex(1, 0.0, 0.0);
    graph.addVertex(2, 1.0, 1.0);
    graph.addVertex(3, 2.0, 2.0);
    graph.addVertex(4, 1.0, 2.0);

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

    return 0;
}
