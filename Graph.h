#ifndef L4_GRAPH_H
#define L4_GRAPH_H

#include "ArraySequence.h"
#include "Dictionary.h"
#include "PriorityQueue.h"
#include <limits>
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
        if(!this->getVertices().contains_key(from) || !this->getVertices().contains_key(to)){
            throw std::invalid_argument("Start or end vertex does not exist in the graph");
        }
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

    void visualize(const std::string& filename, const ArraySequence<Edge>& highlightedEdges = {}) const {
        std::ofstream outFile(filename);

        outFile << "digraph G {\n"; // Направленный граф

        // Создаем вершины
        for (const auto& pair : vertices) {
            if (pair.value.id != -1) {
                outFile << "  " << pair.value.id << " [label=\"" << pair.value.content << "\"];\n";
            }
        }

        // Создаем рёбра
        for (const auto& pair : edges) {
            if (pair.value == nullptr) {
                continue;
            }
            for (const auto& edge : *pair.value) {
                bool isHighlighted = std::find_if(
                    highlightedEdges.begin(), highlightedEdges.end(),
                    [&edge](const Edge& e) {
                        return e.from == edge.from && e.to == edge.to && e.content == edge.content;
                    }) != highlightedEdges.end();

                outFile << "  " << edge.from << " -> " << edge.to
                        << " [label=\"" << edge.content << "\""
                        << (isHighlighted ? ", color=red, penwidth=2.0" : "")
                        << "];\n";
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
        if (!graph.getVertices().contains_key(start) || !graph.getVertices().contains_key(end)) {
            throw std::invalid_argument("Start or end vertex does not exist in the graph");
        }

        Dictionary<int, double> distances;
        Dictionary<int, int> previous;

        for (const auto& pair : graph.getVertices()) {
            distances[pair.key] = std::numeric_limits<double>::infinity();
            previous[pair.key] = -1;
        }
        distances[start] = 0;

        using P = std::pair<double, int>;
        PriorityQueue<P> pq;
        pq.append({0, start});

        while (!pq.empty()) {
            auto [currentDistance, currentVertex] = pq.getFirst();
            pq.pop();

            if (currentDistance > distances[currentVertex]) {
                continue;
            }

            const auto* edges = graph.getOutgoingEdges(currentVertex);
            if (edges != nullptr) {
                for (const auto& edge : *edges) {
                    double newDist = currentDistance + edge.content;
                    if (newDist < distances[edge.to]) {
                        distances[edge.to] = newDist;
                        previous[edge.to] = currentVertex;
                        pq.append({newDist, edge.to});
                    }
                }
            }
        }

        ArraySequence<int> path;
        for (int at = end; at != -1; at = previous[at]) {
            path.append(at);
        }
        std::reverse(path.begin(), path.end());

        if (path.getLength() == 1 && path[0] != start) {
            return {std::numeric_limits<double>::infinity(), {}};
        }

        return {distances[end], path};
    }

};

#endif //L4_GRAPH_H
