//
// Created by Admin on 28.04.2024.
//

#ifndef SEQUENCES_SEQUENCE_H
#define SEQUENCES_SEQUENCE_H

//чисто виртуальный класс (абстрактный класс) - класс, который содержит хотя бы одну чисто виртуальную функцию
//Он нужен нам для того, чтобы создать интерфейс, который будут реализовывать другие классы
//Простыми словами он просто показывает, какие функции нужно реализовать в классе, который будет наследоваться от него
//T - любой тип данных, будь то числа, строки и т.д.

#include <functional>

template<class T>
class Sequence {
public:
    Sequence() = default; //конструктор по умолчанию

    virtual ~Sequence() = default; //деструктор по умолчанию

    //чисто виртуальная функция - функция, которая не имеет реализации в этом классе
    virtual T getFirst() const = 0; //функция, которая возвращает первый элемент последовательности

    virtual T getLast() const = 0; //функция, которая возвращает последний элемент последовательности

    virtual T& get(int index) const = 0; //функция, которая возвращает элемент последовательности по индексу

    virtual int getLength() const = 0; //функция, которая возвращает длину последовательности

    virtual Sequence<T> *getSubsequence(int startIndex, int endIndex) const = 0; //функция, которая возвращает подпоследовательность

    virtual Sequence<T>* append(T value) = 0; //функция, которая добавляет элемент в конец последовательности

    virtual Sequence<T>* prepend(T value) = 0; //функция, которая добавляет элемент в начало последовательности

    virtual Sequence<T>* insertAt(T value, int index) = 0; //функция, которая добавляет элемент в последовательность по индексу

    virtual Sequence<T>* set(T value, int index) = 0; //функция, которая изменяет элемент в последовательности по индексу

    virtual Sequence<T> *concat(Sequence<T> *sequence) const = 0; //функция, которая объединяет две последовательности

    //virtual void print() const = 0; //функция, которая выводит последовательность на экран

    virtual T& operator[](int index) const = 0; //перегрузка оператора [] для удобства работы с последовательностью

    virtual void clear() = 0; //функция, которая очищает последовательность

    virtual Sequence<T>* copy() const = 0; //функция, которая создает копию последовательности

    //map - функция, которая применяет функцию f к каждому элементу последовательности
    //std::function - класс, который позволяет передавать функции как аргументы другой функции
    friend void map(std::function<T(T)> f, Sequence<T> *sequence) {
        //проходим по всем элементам последовательности и применяем к ним функцию f
        for (int i = 0; i < sequence->getLength(); i++) {
            sequence->set(f(sequence->get(i)), i);
        }
    }

    //reduce - функция, которая применяет функцию f к каждому элементу последовательности,
    //но в отличие от map, она принимает начальное значение init
    friend void reduce(std::function<T(T, T)> f, Sequence<T> *sequence, T init) {
        //проходим по всем элементам последовательности и применяем к ним функцию f
        for (int i = 0; i < sequence->getLength(); i++) {
            init = f(init, sequence->get(i));
        }
    }
};


#endif //SEQUENCES_SEQUENCE_H
