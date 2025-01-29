//
// Created by Admin on 28.04.2024.
//

#ifndef SEQUENCES_ARRAYSEQUENCE_H
#define SEQUENCES_ARRAYSEQUENCE_H

#include "Sequence.h"
#include <stdexcept>
#include "DynamicArray.h"
#include <fstream>

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

    ArraySequence(int capacity){
        this->arrayList = new DynamicArray<T>(capacity);
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

    /*
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
     */

    //Функция, которая возвращает массив, который хранит элементы последовательности
    DynamicArray<T>* getArray() const {
        return this->arrayList;
    }

    //Оператор [], который возвращает элемент последовательности по индексу
    T& operator[](int index) const override {
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

    T removeLast() {
        if (this->arrayList->getLength() == 0) {
            throw std::out_of_range("ArraySequence is empty");
        }
        T last = this->getLast();
        this->remove(this->arrayList->getLength() - 1);
        return last;
    }

    int getCapacity() const {
        return this->arrayList->getCapacity();
    }

    class Iterator {
    private:
        ArraySequence<T> *arraySequence;
        int index;
    public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = T *;
        using reference = T &;

        Iterator(ArraySequence<T> *arraySequence, int index) : arraySequence(arraySequence), index(index) {}

        Iterator(const Iterator &it) : arraySequence(it.arraySequence), index(it.index) {}

        Iterator &operator++() {
            index++;
            return *this;
        }

        Iterator operator++(int) {
            Iterator it = *this;
            index++;
            return it;
        }

        Iterator &operator--() {
            index--;
            return *this;
        }

        Iterator operator--(int) {
            Iterator it = *this;
            index--;
            return it;
        }

        Iterator &operator+=(difference_type n) {
            index += n;
            return *this;
        }

        Iterator operator+(difference_type n) const {
            return Iterator(arraySequence, index + n);
        }

        Iterator &operator-=(difference_type n) {
            index -= n;
            return *this;
        }

        Iterator operator-(difference_type n) const {
            return Iterator(arraySequence, index - n);
        }

        difference_type operator-(const Iterator &it) const {
            return index - it.index;
        }

        bool operator==(const Iterator &it) const {
            return index == it.index;
        }

        bool operator!=(const Iterator &it) const {
            return index != it.index;
        }

        bool operator<(const Iterator &it) const {
            return index < it.index;
        }

        bool operator>(const Iterator &it) const {
            return index > it.index;
        }

        bool operator<=(const Iterator &it) const {
            return index <= it.index;
        }

        bool operator>=(const Iterator &it) const {
            return index >= it.index;
        }

        T &operator*() {
            return arraySequence->get(index);
        }

        T *operator->() {
            return &arraySequence->get(index);
        }
    };

    class ConstIterator {
    private:
        const ArraySequence<T> *arraySequence;
        int index;
    public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = T;
        using difference_type = std::ptrdiff_t;
        using pointer = const T *;
        using reference = const T &;

        ConstIterator(const ArraySequence<T> *arraySequence, int index) : arraySequence(arraySequence), index(index) {}

        ConstIterator(const ConstIterator &it) : arraySequence(it.arraySequence), index(it.index) {}

        ConstIterator &operator++() {
            index++;
            return *this;
        }

        ConstIterator operator++(int) {
            ConstIterator it = *this;
            index++;
            return it;
        }

        ConstIterator &operator--() {
            index--;
            return *this;
        }

        ConstIterator operator--(int) {
            ConstIterator it = *this;
            index--;
            return it;
        }

        ConstIterator &operator+=(difference_type n) {
            index += n;
            return *this;
        }

        ConstIterator operator+(difference_type n) const {
            return ConstIterator(arraySequence, index + n);
        }

        ConstIterator &operator-=(difference_type n) {
            index -= n;
            return *this;
        }

        ConstIterator operator-(difference_type n) const {
            return ConstIterator(arraySequence, index - n);
        }

        difference_type operator-(const ConstIterator &it) const {
            return index - it.index;
        }

        bool operator==(const ConstIterator &it) const {
            return index == it.index;
        }

        bool operator!=(const ConstIterator &it) const {
            return index != it.index;
        }

        bool operator<(const ConstIterator &it) const {
            return index < it.index;
        }

        bool operator>(const ConstIterator &it) const {
            return index > it.index;
        }

        bool operator<=(const ConstIterator &it) const {
            return index <= it.index;
        }

        bool operator>=(const ConstIterator &it) const {
            return index >= it.index;
        }

        const T &operator*() const {
            return arraySequence->get(index);
        }

        const T *operator->() const {
            return &arraySequence->get(index);
        }
    };

    Iterator begin() {
        return Iterator(this, 0);
    }

    Iterator end() {
        return Iterator(this, this->getLength());
    }

    ConstIterator begin() const {
        return ConstIterator(this, 0);
    }

    ConstIterator end() const {
        return ConstIterator(this, this->getLength());
    }

    ~ArraySequence() {
        //std::cout << "Deleting ArraySequence, length: " << arrayList->getLength() << std::endl;
        if (this->arrayList != nullptr) {
            delete this->arrayList;
        }
    }

};

#endif //SEQUENCES_ARRAYSEQUENCE_H
