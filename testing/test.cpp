#include "gtest/gtest.h"
#include "../ArraySequence.h"
#include "../PriorityQueue.h"
#include "../Dictionary.h"

TEST(ArraySequence, Constructor) {
    ArraySequence<int> sequence;
    ASSERT_EQ(sequence.getLength(), 0);
}

TEST(ArraySequence, Append) {
    ArraySequence<int> sequence;
    sequence.append(1);
    sequence.append(2);
    sequence.append(3);
    ASSERT_EQ(sequence.getLength(), 3);
    ASSERT_EQ(sequence[0], 1);
    ASSERT_EQ(sequence[1], 2);
    ASSERT_EQ(sequence[2], 3);
}

TEST(ArraySequence, Prepend) {
    ArraySequence<int> sequence;
    sequence.prepend(1);
    sequence.prepend(2);
    sequence.prepend(3);
    ASSERT_EQ(sequence.getLength(), 3);
    ASSERT_EQ(sequence[0], 3);
    ASSERT_EQ(sequence[1], 2);
    ASSERT_EQ(sequence[2], 1);
}

TEST(ArraySequence, InsertAt) {
    ArraySequence<int> sequence;
    sequence.append(1);
    sequence.append(2);
    sequence.append(3);
    sequence.insertAt(4, 1);
    ASSERT_EQ(sequence.getLength(), 4);
    ASSERT_EQ(sequence[0], 1);
    ASSERT_EQ(sequence[1], 4);
    ASSERT_EQ(sequence[2], 2);
    ASSERT_EQ(sequence[3], 3);
}

TEST(ArraySequence, GetSubsequence) {
    ArraySequence<int> sequence;
    sequence.append(1);
    sequence.append(2);
    sequence.append(3);
    auto subsequence = sequence.getSubsequence(1, 2);
    ASSERT_EQ(subsequence->getLength(), 2);
    ASSERT_EQ(subsequence->get(0), 2);
    ASSERT_EQ(subsequence->get(1), 3);
}

TEST(ArraySequence, Set) {
    ArraySequence<int> sequence;
    sequence.append(1);
    sequence.append(2);
    sequence.append(3);
    sequence.set(4, 1);
    ASSERT_EQ(sequence.getLength(), 3);
    ASSERT_EQ(sequence[0], 1);
    ASSERT_EQ(sequence[1], 4);
    ASSERT_EQ(sequence[2], 3);
}

TEST(ArraySequence, Copy) {
    ArraySequence<int> sequence;
    sequence.append(1);
    sequence.append(2);
    sequence.append(3);
    auto copy = sequence.copy();
    ASSERT_EQ(copy->getLength(), 3);
    ASSERT_EQ(copy->get(0), 1);
    ASSERT_EQ(copy->get(1), 2);
    ASSERT_EQ(copy->get(2), 3);
}

TEST(ArraySequence, Clear) {
    ArraySequence<int> sequence;
    sequence.append(1);
    sequence.append(2);
    sequence.append(3);
    sequence.clear();
    ASSERT_EQ(sequence.getLength(), 0);
}

TEST(ArraySequence, Iterator){
    ArraySequence<int> sequence;
    sequence.append(1);
    sequence.append(2);
    sequence.append(3);
    auto it = sequence.begin();
    ASSERT_EQ(*it, 1);
    ++it;
    ASSERT_EQ(*it, 2);
    ++it;
    ASSERT_EQ(*it, 3);
    ++it;
    ASSERT_EQ(it, sequence.end());
}

TEST(PriorityQueue, Constructor) {
    PriorityQueue<int> queue;
    ASSERT_EQ(queue.getLength(), 0);
}

TEST(PriorityQueue, Append) {
    PriorityQueue<int> queue;
    queue.append(3);
    queue.append(2);
    queue.append(1);
    ASSERT_EQ(queue.getLength(), 3);
}

TEST(PriorityQueue, Pop) {
    PriorityQueue<int, ArraySequence<int>, std::less<>> queue;
    queue.append(3);
    queue.append(2);
    queue.append(1);
    ASSERT_EQ(queue.pop(), 1);
    ASSERT_EQ(queue.pop(), 2);
    ASSERT_EQ(queue.pop(), 3);
    ASSERT_EQ(queue.getLength(), 0);
}

TEST(PriorityQueue, Copy) {
    PriorityQueue<int, ArraySequence<int>, std::less<>> queue;
    queue.append(3);
    queue.append(2);
    queue.append(1);
    auto copy = queue.copy();
    ASSERT_EQ(copy->getLength(), 3);
    ASSERT_EQ(copy->pop(), 1);
    ASSERT_EQ(copy->pop(), 2);
    ASSERT_EQ(copy->pop(), 3);
    ASSERT_EQ(copy->getLength(), 0);
}

TEST(PriorityQueue, Clear) {
    PriorityQueue<int> queue;
    queue.append(3);
    queue.append(2);
    queue.append(1);
    queue.clear();
    ASSERT_EQ(queue.getLength(), 0);
}

TEST(PriorityQueue, Empty) {
    PriorityQueue<int> queue;
    ASSERT_TRUE(queue.empty());
    queue.append(1);
    ASSERT_FALSE(queue.empty());
    queue.pop();
    ASSERT_TRUE(queue.empty());
}

TEST(PriorityQueue, Insert){
    PriorityQueue<int> queue;
    ASSERT_THROW(queue.insertAt(1, 0), std::logic_error);
}

TEST(PriorityQueue, Get){
    PriorityQueue<int> queue;
    ASSERT_THROW(queue.get(0), std::logic_error);
}

TEST(Dictionary, Add){
    Dictionary<int, std::string> dict;
    dict.add(1, "one");
    dict.add(2, "two");
    dict.add(3, "three");
    dict.add(4, "four");
    dict.add(5, "five");
    ASSERT_EQ(dict.get(1), "one");
    ASSERT_EQ(dict.get(2), "two");
    ASSERT_EQ(dict.get(3), "three");
    ASSERT_EQ(dict.get(4), "four");
    ASSERT_EQ(dict.get(5), "five");
}

TEST(Dictionary, Remove){
    Dictionary<int, std::string> dict;
    dict.add(1, "one");
    dict.add(2, "two");
    dict.add(3, "three");
    dict.add(4, "four");
    dict.add(5, "five");
    dict.remove(3);
    ASSERT_EQ(dict.get(1), "one");
    ASSERT_EQ(dict.get(2), "two");
    ASSERT_THROW(dict.get(3), std::runtime_error);
    ASSERT_EQ(dict.get(4), "four");
    ASSERT_EQ(dict.get(5), "five");
}

TEST(Dictionary, Contains){
    Dictionary<int, std::string> dict;
    dict.add(1, "one");
    dict.add(2, "two");
    dict.add(3, "three");
    dict.add(4, "four");
    dict.add(5, "five");
    ASSERT_TRUE(dict.contains_key(1));
    ASSERT_TRUE(dict.contains_key(2));
    ASSERT_TRUE(dict.contains_key(3));
    ASSERT_TRUE(dict.contains_key(4));
    ASSERT_TRUE(dict.contains_key(5));
    dict.remove(1);
    dict.remove(3);
    dict.remove(5);
    ASSERT_FALSE(dict.contains_key(1));
    ASSERT_TRUE(dict.contains_key(2));
    ASSERT_FALSE(dict.contains_key(3));
    ASSERT_TRUE(dict.contains_key(4));
    ASSERT_FALSE(dict.contains_key(5));
}

TEST(Dictionary, Get){
    Dictionary<int, std::string> dict;
    dict.add(1, "one");
    dict.add(2, "two");
    dict.add(3, "three");
    dict.add(4, "four");
    dict.add(5, "five");
    ASSERT_EQ(dict.get(1), "one");
    ASSERT_EQ(dict.get(2), "two");
    ASSERT_EQ(dict.get(3), "three");
    ASSERT_EQ(dict.get(4), "four");
    ASSERT_EQ(dict.get(5), "five");
    dict.remove(1);
    dict.remove(3);
    dict.remove(5);
    ASSERT_THROW(dict.get(1), std::runtime_error);
    ASSERT_EQ(dict.get(2), "two");
    ASSERT_THROW(dict.get(3), std::runtime_error);
    ASSERT_EQ(dict.get(4), "four");
    ASSERT_THROW(dict.get(5), std::runtime_error);
}

TEST(Dictionary, Clear){
    Dictionary<int, std::string> dict;
    dict.add(1, "one");
    dict.add(2, "two");
    dict.add(3, "three");
    dict.add(4, "four");
    dict.add(5, "five");
    dict.clear();
    ASSERT_THROW(dict.get(1), std::runtime_error);
    ASSERT_THROW(dict.get(2), std::runtime_error);
    ASSERT_THROW(dict.get(3), std::runtime_error);
    ASSERT_THROW(dict.get(4), std::runtime_error);
    ASSERT_THROW(dict.get(5), std::runtime_error);
}

#include "../Graph.h"
#include <random>
#include <fstream>

// Вспомогательные функции
char vertexIdToChar(int id) {
    return static_cast<char>('A' + id - 1);
}

int charToVertexId(char c) {
    return static_cast<int>(c - 'A' + 1);
}

bool fileExists(const std::string& filename) {
    std::ifstream file(filename);
    return file.good();
}

// Генерация графа с 20 вершинами и случайными рёбрами
Graph<std::string, double> generateRandomGraphWithGuaranteedPath() {
    Graph<std::string, double> graph;

    // Добавляем 15 вершин
    for (int i = 1; i <= 15; ++i) {
        graph.addVertex(i, std::string(1, vertexIdToChar(i)));
    }

    // Генератор случайных рёбер
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> vertexDist(1, 15);  // Выбор вершин
    std::uniform_real_distribution<> weightDist(21.0, 50.0); // Случайные веса

    // Добавляем случайные рёбра
    for (int i = 0; i < 20; ++i) {
        int from = vertexDist(gen);
        int to = vertexDist(gen);
        if (from != to) {
            double weight = weightDist(gen);
            graph.addEdge(from, to, weight);
        }
    }

    // Гарантируем путь от A до F с длиной 20
    graph.addEdge(charToVertexId('A'), charToVertexId('B'), 5.0);
    graph.addEdge(charToVertexId('B'), charToVertexId('C'), 5.0);
    graph.addEdge(charToVertexId('C'), charToVertexId('F'), 10.0);

    return graph;
}

// Тест на кратчайший путь
TEST(GraphTests, ShortestPathVisualizationAndCheck) {
    // Генерация графа
    auto graph = generateRandomGraphWithGuaranteedPath();

    // Проверяем кратчайший путь
    auto result = ShortestPath::dijkstra(graph, charToVertexId('A'), charToVertexId('F'));

    // Ожидаемое расстояние
    ASSERT_DOUBLE_EQ(result.distance, 20.0);

    // Проверяем правильность пути
    ASSERT_EQ(result.path.getLength(), 4);
    EXPECT_EQ(vertexIdToChar(result.path[0]), 'A');
    EXPECT_EQ(vertexIdToChar(result.path[1]), 'B');
    EXPECT_EQ(vertexIdToChar(result.path[2]), 'C');
    EXPECT_EQ(vertexIdToChar(result.path[3]), 'F');

    // Подсветка кратчайшего пути для визуализации
    ArraySequence<Graph<std::string, double>::Edge> highlightedEdges;
    for (int i = 0; i < result.path.getLength() - 1; ++i) {
        int from = result.path[i];
        int to = result.path[i + 1];
        const auto* edges = graph.getOutgoingEdges(from);
        if (edges != nullptr) {
            for (const auto& edge : *edges) {
                if (edge.to == to) {
                    highlightedEdges.append(edge);
                    break;
                }
            }
        }
    }

    // Генерация визуализации
    graph.visualize("graph_test.dot", highlightedEdges);

    // Проверяем, что файл визуализации был создан
    EXPECT_TRUE(fileExists("graph_test.dot"));

    // Генерация изображения графа через Graphviz
    std::system("dot -Tpng graph_test.dot -o graph_test.png");

    // Проверяем, что изображение было создано
    EXPECT_TRUE(fileExists("graph_test.png"));
}
