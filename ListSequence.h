//
// Created by Admin on 28.04.2024.
//

#ifndef SEQUENCES_LISTSEQUENCE_H
#define SEQUENCES_LISTSEQUENCE_H

#include "Sequence.h"
#include <stdexcept>
#include <iostream>
#include "LinkedList.h"

//класс ListSequence - класс, который реализует последовательность на основе двусвязного списка
template<class T>
class ListSequence : public Sequence<T> {
private:
    LinkedList<T>* linkedList; //указатель на двусвязный список
public:
    //Конструктор, который создает последовательность из массива
    ListSequence(T* items, int count){
        this->linkedList = new LinkedList<T>(items, count);
    }

    //Конструктор по умолчанию - создает пустую последовательность (использует конструктор LinkedList)
    ListSequence(){
        this->linkedList = new LinkedList<T>();
    }

    //Конструктор копирования - создает копию последовательности (использует конструктор LinkedList)
    ListSequence(const ListSequence<T> &listSequence){
        this->linkedList = new LinkedList<T>(*listSequence.linkedList);
    }

    //Функция, которая возвращает первый элемент последовательности
    T getFirst() const override {
        return this->linkedList->getFirst();
    }

    //Функция, которая возвращает последний элемент последовательности
    T getLast() const override {
        return this->linkedList->getLast();
    }

    //Функция, которая возвращает элемент последовательности по индексу index
    T& get(int index) const override {
        return this->linkedList->get(index);
    }

    //Функция, которая возвращает подпоследовательность от startIndex до endIndex
    ListSequence<T> *getSubsequence(int startIndex, int endIndex) const override {
        if (startIndex < 0 || startIndex >= this->linkedList->getLength() || endIndex < 0 || endIndex >= this->linkedList->getLength() || startIndex > endIndex) {
            throw std::out_of_range("Index out of range");
        }
        T *items = new T[endIndex - startIndex + 1];
        for (int i = startIndex; i <= endIndex; i++) {
            items[i - startIndex] = this->linkedList->get(i);
        }
        return new ListSequence<T>(items, endIndex - startIndex + 1);
    }

    //Функция, которая возвращает длину последовательности
    int getLength() const override {
        return this->linkedList->getLength();
    }

    //Функция, которая добавляет элемент в конец последовательности
    Sequence<T>* append(T value) override {
        this->linkedList->append(value);
        return this;
    }

    //Функция, которая добавляет элемент в начало последовательности
    Sequence<T>* prepend(T value) override {
        this->linkedList->prepend(value);
        return this;
    }

    //Функция, которая добавляет элемент в последовательность по индексу index
    Sequence<T>* insertAt(T value, int index) override {
        this->linkedList->insertAt(value, index);
        return this;
    }

    //Функция, которая изменяет элемент в последовательности по индексу
    Sequence<T>* set(T value, int index) override {
        this->linkedList->set(value, index);
        return this;
    }

    //Функция, которая объединяет две последовательности
    ListSequence<T> *concat(Sequence<T> *sequence) const override {
        auto *newList = new ListSequence<T>(*this);
        for (int i = 0; i < sequence->getLength(); i++) {
            newList->append(sequence->get(i));
        }
        return newList;
    }

    //Функция, которая выводит последовательность на экран
    void print() const override {
        std::cout << "[";
        for (int i = 0; i < this->linkedList->getLength(); i++) {
            std::cout << this->linkedList->get(i);
            if (i != this->linkedList->getLength() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "]" << std::endl;
    }

    //Функция, которая возвращает указатель на двусвязный список
    LinkedList<T>* getLinkedList() const {
        return this->linkedList;
    }

    //Оператор [], который возвращает элемент последовательности по индексу index
    T operator[](int index) const override {
        return this->linkedList->get(index);
    }

    //Функция, которая очищает последовательность
    void clear() override {
        this->linkedList->clear();
    }

    //Функция, которая создает копию последовательности
    ListSequence<T>* copy() const override {
        return new ListSequence<T>(*this);
    }

    //Деструктор
    ~ListSequence() override {
        delete this->linkedList;
    }
};

#endif //SEQUENCES_LISTSEQUENCE_H
