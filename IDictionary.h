#ifndef L3_IDICTIONARY_H
#define L3_IDICTIONARY_H

// Интерфейс IDictionary
template <typename Key, typename Value>
class IDictionary {
public:
    virtual size_t count() const = 0; // Количество элементов в словаре
    virtual size_t capacity() const = 0; // Вместимость словаря
    virtual bool contains_key(const Key& key) const = 0; // Проверка наличия ключа
    virtual void add(const Key& key, const Value& value) = 0; // Добавление элемента
    virtual void remove(const Key& key) = 0; // Удаление элемента
    virtual Value& get(const Key& key) = 0; // Получение значения по ключу
    virtual const Value& get(const Key& key) const = 0; // Получение значения по ключу (константная версия)
    virtual Value& operator[](const Key& key) = 0; // Оператор доступа по ключу
    //virtual const Value& operator[](const Key& key) const = 0; // Оператор доступа по ключу (константная версия)
    virtual void clear() = 0; // Очистка словаря
    virtual ~IDictionary() = default; // Виртуальный деструктор (для возможности наследования)
};

#endif //L3_IDICTIONARY_H