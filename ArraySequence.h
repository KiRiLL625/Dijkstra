//
// Created by Admin on 28.04.2024.
//

#ifndef SEQUENCES_ARRAYSEQUENCE_H
#define SEQUENCES_ARRAYSEQUENCE_H

#include "Sequence.h"
#include <stdexcept>
#include "DynamicArray.h"

std::ostream& operator<<(std::ostream& os, const std::pair<double, int>& p) {
    os << "(" << p.first << ", " << p.second << ")";
    return os;
}

//класс ArraySequence - класс, который реализует последовательность на основе обычного массива
//Наследует класс Sequence, то есть мы должны реализовать все методы, которые в нем объявлен
//Класс использует функции из класса DynamicArray для работы с массивом для скрытия реализации
template<class T>
class ArraySequence : public Sequence<T> {
protected:
    DynamicArray<T> *arrayList; //указатель на массив
public:
    //Конструктор, который создает последовательность из массива
    ArraySequence(T* items, int count){
        //просто используем конструктор DynamicArray, чтобы создать массив из элементов
        this->arrayList = new DynamicArray<T>(items, count);
    }

    //Конструктор по умолчанию - создает пустую последовательность
    ArraySequence(){
        //просто используем конструктор DynamicArray, чтобы создать пустой массив
        this->arrayList = new DynamicArray<T>();
    }

    //Конструктор копирования - создает копию последовательности
    ArraySequence(const ArraySequence<T> &arraySequence){
        //просто используем конструктор DynamicArray, чтобы создать копию массива
        this->arrayList = new DynamicArray<T>(*arraySequence.arrayList);
    }

    //Функция, которая возвращает первый элемент последовательности
    T getFirst() const override {
        return this->arrayList->get(0); //просто возвращаем первый элемент массива
    }

    //Функция, которая возвращает последний элемент последовательности
    T getLast() const override {
        return this->arrayList->get(this->arrayList->getLength() - 1);
    }

    //Функция, которая возвращает элемент последовательности по индексу index
    T& get(int index) const override {
        return this->arrayList->get(index);
    }

    //Функция, которая возвращает подпоследовательность от startIndex до endIndex
    ArraySequence<T> *getSubsequence(int startIndex, int endIndex) const override {
        if (startIndex < 0 || startIndex >= this->arrayList->getLength() || endIndex < 0 || endIndex >= this->arrayList->getLength() || startIndex > endIndex) {
            throw std::out_of_range("Index out of range");
        }
        T *items = new T[endIndex - startIndex + 1];
        for (int i = startIndex; i <= endIndex; i++) {
            items[i - startIndex] = this->arrayList->get(i);
        }
        return new ArraySequence<T>(items, endIndex - startIndex + 1);
    }

    //Функция, которая возвращает длину последовательности
    int getLength() const override {
        return this->arrayList->getLength();
    }

    //Функция, которая добавляет элемент в конец последовательности
    Sequence<T>* append(T value) override {
        this->arrayList->append(value);
        return this;
    }

    //Функция, которая добавляет элемент в начало последовательности
    Sequence<T>* prepend(T value) override {
        this->arrayList->prepend(value);
        return this;
    }

    //Функция, которая добавляет элемент в последовательность по индексу
    Sequence<T>* insertAt(T value, int index) override {
        this->arrayList->insertAt(value, index);
        return this;
    }

    //Функция, которая изменяет элемент в последовательности по индексу
    Sequence<T>* set(T value, int index) override {
        this->arrayList->set(index, value);
        return this;
    }

    //Функция, которая объединяет две последовательности
    ArraySequence<T> *concat(Sequence<T> *sequence) const override {
        //создаем новую последовательность, которая является копией текущей
        ArraySequence<T> *new_array = new ArraySequence<T>(*this);
        //добавляем все элементы из второй последовательности в новую последовательность
        for (int i = 0; i < sequence->getLength(); i++) {
            new_array->append(sequence->get(i));
        }
        return new_array;
    }

    //Функция, которая выводит последовательность на экран
    void print() const override {
        std::cout << "[";
        for (int i = 0; i < this->arrayList->getLength(); i++) {
            std::cout << this->arrayList->get(i);
            if (i != this->arrayList->getLength() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "]" << std::endl;
    }

    //Функция, которая возвращает массив, который хранит элементы последовательности
    DynamicArray<T>* getArray() const {
        return this->arrayList;
    }

    //Оператор [], который возвращает элемент последовательности по индексу
    T operator[](int index) const override {
        return this->arrayList->get(index);
    }

    //Функция, которая очищает последовательность
    void clear() override {
        this->arrayList->clear();
    }

    //Функция, которая создает копию последовательности
    ArraySequence<T>* copy() const override {
        return new ArraySequence<T>(*this);
    }

    ArraySequence<T>* remove(int index) {
        this->arrayList->removeAt(index);
        return this;
    }

    T* begin() {
        return arrayList->begin();
    }

    T* end() {
        return arrayList->end();
    }

    const T* begin() const {
        return arrayList->begin();
    }

    const T* end() const {
        return arrayList->end();
    }

    ~ArraySequence() { //деструктор
        delete this->arrayList;
    }
};

#endif //SEQUENCES_ARRAYSEQUENCE_H
