#ifndef L4_PRIORITYQUEUE_H
#define L4_PRIORITYQUEUE_H

#include "Sequence.h"
#include "ArraySequence.h"

template<typename T, typename Container = ArraySequence<T>, typename Compare = std::greater<T>>
class PriorityQueue: public Sequence<T> {
private:
    Container *container; //контейнер, который хранит элементы
    Compare compare; //функция сравнения элементов
public:
    PriorityQueue() {
        container = new Container();
    }

    PriorityQueue(T* items, int count) {
        container = new Container(items, count);
    }

    PriorityQueue(const PriorityQueue<T, Container, Compare> &priorityQueue) {
        container = new Container(*priorityQueue.container);
    }

    T getFirst() const override {
        return container->getFirst();
    }

    T getLast() const override {
        throw std::logic_error("Not implemented");
    }

    T& get(int index) const override {
        throw std::logic_error("Not implemented");
    }

    int getLength() const override {
        return container->getLength();
    }

    PriorityQueue<T, Container, Compare> *getSubsequence(int startIndex, int endIndex) const override {
        throw std::logic_error("Not implemented");
    }

    PriorityQueue<T, Container, Compare> *append(T value) override {
        container->append(value); //добавляем элемент в конец
        int i = container->getLength() - 1; //запоминаем индекс добавленного элемента
        // Пока элемент больше (или меньше, в зависимости от функции сравнения) своего родителя, меняем их местами
        while (i > 0 && compare(container->get(i), container->get((i - 1) / 2))) {
            std::swap(container->get(i), container->get((i - 1) / 2)); //меняем элементы местами (делим на 2, чтобы найти индекс родителя)
            i = (i - 1) / 2; //переходим к родителю
        }
        return this; //возвращаем указатель на текущий объект
    }

    PriorityQueue<T, Container, Compare> *prepend(T value) override {
        throw std::logic_error("Not implemented");
    }

    PriorityQueue<T, Container, Compare> *insertAt(T value, int index) override {
        throw std::logic_error("Not implemented");
    }

    PriorityQueue<T, Container, Compare> *set(T value, int index) override {
        throw std::logic_error("Not implemented");
    }

    PriorityQueue<T, Container, Compare> *concat(Sequence<T> *sequence) const override {
        throw std::logic_error("Not implemented");
    }

    void print() const override {
        container->print();
    }

    T operator[](int index) const override {
        throw std::logic_error("Not implemented");
    }

    void clear() override {
        container->clear();
    }

    PriorityQueue<T, Container, Compare> *copy() const override {
        return new PriorityQueue<T, Container, Compare>(*this);
    }

    bool empty() const {
        return container->getLength() == 0;
    }

    T pop() { //удаление элемента с наивысшим приоритетом
        if (empty()) {
            throw std::runtime_error("Queue is empty");
        }
        T result = container->getFirst(); //запоминаем элемент с наивысшим приоритетом
        container->set(container->get(container->getLength() - 1), 0); //перемещаем последний элемент на место первого
        container->remove(container->getLength() - 1); //удаляем последний элемент
        int i = 0; //запоминаем индекс текущего элемента
        while (2 * i + 1 < container->getLength()) { //пока у текущего элемента есть потомки
            int left = 2 * i + 1; //левый потомок
            int right = 2 * i + 2; //правый потомок
            int j = left; //запоминаем индекс потомка
            // Если правый потомок существует и он меньше (или больше, в зависимости от функции сравнения) левого потомка
            if (right < container->getLength() && compare(container->get(right), container->get(left))) {
                j = right; //переходим к правому потомку
            }
            // Если текущий элемент меньше (или больше, в зависимости от функции сравнения) потомка, то меняем их местами
            if (compare(container->get(i), container->get(j))) {
                break;
            }
            std::swap(container->get(i), container->get(j)); //меняем элементы местами
            i = j; //переходим к потомку
        }
        return result; //возвращаем удаленный элемент
    }
};

#endif //L4_PRIORITYQUEUE_H
