//
// Created by Admin on 09.05.2024.
//

#ifndef SEQUENCES_LINKEDLIST_H
#define SEQUENCES_LINKEDLIST_H

#include <stdexcept>

//класс LinkedList - класс, который реализует двусвязный список
template<class T>
class LinkedList {
private:
    //структура Node - структура, которая представляет узел списка
    struct Node {
        T value; //значение узла
        Node* next; //указатель на следующий узел
        Node* prev; //указатель на предыдущий узел
        //Конструктор узла - создает узел с заданным значением
        explicit Node(T value) : value(value), next(nullptr), prev(nullptr) {}
        void setNext(Node* next) {
            this->next = next;
        }
        ~Node() = default;
    };

    Node* head; //указатель на начало списка
    Node* tail; //указатель на конец списка
    int length; //длина списка
public:
    //Конструктор, который создает список из массива
    LinkedList(T* items, int count) {
        this->length = 0;
        this->head = nullptr;
        this->tail = nullptr;
        for (int i = 0; i < count; i++) {
            this->append(items[i]);
        }
    }

    //Конструктор по умолчанию - создает пустой список
    LinkedList(){
        this->length = 0;
        this->head = nullptr;
        this->tail = nullptr;
    }

    //Конструктор копирования - создает копию списка
    LinkedList(const LinkedList<T> &linkedList){
        this->length = 0;
        this->head = nullptr;
        this->tail = nullptr;
        //копируем все элементы из списка, вставляя их в конец нового списка через append()
        Node* current = linkedList.head;
        while (current != nullptr) {
            this->append(current->value);
            current = current->next;
        }
    }

    //Функция, которая возвращает первый элемент списка
    T getFirst() const {
        if (this->length == 0) {
            throw std::out_of_range("List is empty");
        }
        return this->head->value; //возвращаем значение первого узла (head - начало_
    }

    //Функция, которая возвращает последний элемент списка
    T getLast() const {
        if (this->length == 0) {
            throw std::out_of_range("List is empty");
        }
        return this->tail->value; //возвращаем значение последнего узла (tail - конец)
    }

    //Функция, которая возвращает элемент списка по индексу
    T& get(int index) const {
        //если индекс меньше 0 или больше длины списка, то выбрасываем исключение
        if (index < 0 || index >= this->length) {
            throw std::out_of_range("Index out of range");
        }
        Node* current = this->head; //начинаем с начала списка
        //проходим по списку до нужного индекса
        for (int i = 0; i < index; i++) {
            current = current->next;
        }
        return current->value; //возвращаем значение узла
    }

    //Функция, которая возвращает подсписок списка от startIndex до endIndex
    LinkedList<T> *getSubList(int startIndex, int endIndex) const {
        if (startIndex < 0 || startIndex >= this->length || endIndex < 0 || endIndex >= this->length || startIndex > endIndex) {
            throw std::out_of_range("Index out of range");
        }
        //создаем новый список, в который будем добавлять элементы
        T *items = new T[endIndex - startIndex + 1];
        Node* current = this->head; //начинаем с начала списка
        for (int i = 0; i < startIndex; i++) {
            current = current->next;
        }
        //проходим по списку от startIndex до endIndex и добавляем элементы в новый список
        for (int i = startIndex; i <= endIndex; i++) {
            items[i - startIndex] = current->value;
            current = current->next;
        }
        //возвращаем новый список на основе массива items
        return new LinkedList<T>(items, endIndex - startIndex + 1);
    }

    //Функция, которая возвращает длину списка
    int getLength() const {
        return this->length;
    }

    //Функция, которая добавляет элемент в конец списка
    void append(T value) {
        //создаем новый узел с заданным значением
        Node* newNode = new Node(value);
        //если список пустой, то новый узел становится началом и концом списка
        if (this->length == 0) {
            this->head = newNode;
            this->tail = newNode;
        } else { //иначе добавляем новый узел в конец списка
            this->tail->next = newNode;
            newNode->prev = this->tail;
            this->tail = newNode;
        }
        this->length++; //увеличиваем длину списка
    }

    //Функция, которая добавляет элемент в начало списка
    void prepend(T value) {
        //создаем новый узел с заданным значением
        Node* newNode = new Node(value);
        //если список пустой, то новый узел становится началом и концом списка
        if (this->length == 0) {
            this->head = newNode;
            this->tail = newNode;
        } else { //иначе добавляем новый узел в начало списка
            this->head->prev = newNode;
            newNode->next = this->head;
            this->head = newNode;
        }
        this->length++;
    }

    //Функция, которая добавляет элемент в список по индексу
    void insertAt(T value, int index) {
        if (index < 0 || index > this->length) {
            throw std::out_of_range("Index out of range");
        }
        //если индекс равен 0, то добавляем элемент в начало списка
        if (index == 0) {
            this->prepend(value);
        } else if (index == this->length) { //если индекс равен длине списка, то добавляем элемент в конец списка
            this->append(value);
        } else {
            //создаем новый узел с заданным значением
            Node* newNode = new Node(value);
            Node* current = this->head; //начинаем с начала списка
            //проходим по списку до нужного индекса
            for (int i = 0; i < index; i++) {
                current = current->next;
            }
            //добавляем новый узел между текущим и предыдущим узлом
            newNode->next = current;
            newNode->prev = current->prev;
            current->prev->next = newNode;
            current->prev = newNode;
            this->length++;
        }
    }

    //Функция, которая изменяет элемент списка по индексу
    void set(T value, int index) {
        //если индекс меньше 0 или больше длины списка, то выбрасываем исключение
        if (index < 0 || index >= this->length) {
            throw std::out_of_range("Index out of range");
        }
        Node* current = this->head; //начинаем с начала списка
        //проходим по списку до нужного индекса
        for (int i = 0; i < index; i++) {
            current = current->next;
        }
        //изменяем значение узла
        current->value = value;
    }

    //Функция, которая объединяет два списка
    LinkedList<T>* concat(LinkedList<T>* list) const {
        LinkedList<T>* newList = new LinkedList<T>(*this);
        Node* current = list->head; //начинаем с начала второго списка
        while (current != nullptr) {
            newList->append(current->value);
            current = current->next;
        }
        return newList;
    }

    Node* getFirstNode() const {
        return this->head;
    }

    Node* getLastNode() const {
        return this->tail;
    }

    void setSize(int size) {
        this->length = size;
    }

    void clear() {
        Node* current = this->head;
        while (current != nullptr) {
            Node* next = current->next;
            delete current;
            current = next;
        }
        this->head = nullptr;
        this->tail = nullptr;
        this->length = 0;
    }

    ~LinkedList() { //деструктор
        Node* current = this->head;
        while (current != nullptr) {
            Node* next = current->next;
            delete current;
            current = next;
        }
    }
};

#endif //SEQUENCES_LINKEDLIST_H
