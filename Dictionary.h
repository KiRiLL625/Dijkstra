#ifndef L3_DICTIONARY_H
#define L3_DICTIONARY_H

#include "IDictionary.h"
#include <vector>
#include <stdexcept>

// Реализация Dictionary
template <typename Key, typename Value>
class Dictionary : public IDictionary<Key, Value> {
private:
    struct KeyValue { // Структура для хранения пары ключ-значение
        Key key;
        Value value;
        bool isOccupied = false; // Флаг, занята ли ячейка
    };

    std::vector<KeyValue> table; // Хеш-таблица
    size_t current_size; // Текущее количество элементов

    size_t hash(const Key& key) const { // Хеш-функция (использует стандартную хеш-функцию для типа Key)
        return std::hash<Key>{}(key) % table.size();
    }

    void rehash() { // Перехеширование
        size_t new_capacity = table.size() * 2; // Увеличиваем вместимость вдвое
        std::vector<KeyValue> new_table(new_capacity); // Создаем новую хеш-таблицу

        for (const auto& entry : table) { // Перебираем все элементы старой таблицы
            if (entry.isOccupied) { // Если ячейка занята
                size_t new_index = std::hash<Key>{}(entry.key) % new_capacity; // Вычисляем новый индекс
                while (new_table[new_index].isOccupied) { // Пока ячейка занята
                    new_index = (new_index + 1) % new_capacity; // Пробуем следующую ячейку
                }
                new_table[new_index] = entry; // Помещаем элемент в новую таблицу
            }
        }

        table = std::move(new_table); // Перемещаем новую таблицу в текущую
    }

public:
    explicit Dictionary(size_t initial_capacity = 16) : table(initial_capacity), current_size(0) {} // Конструктор с параметром по умолчанию

    size_t count() const override { // Количество элементов
        return current_size;
    }

    size_t capacity() const override { // Вместимость
        return table.size();
    }

    bool contains_key(const Key& key) const override { // Проверка наличия ключа
        size_t index = hash(key); // Вычисляем индекс
        size_t start_index = index; // Запоминаем начальный индекс

        do { // Пока не найдем пустую ячейку или ячейку с нужным ключом
            if (!table[index].isOccupied) { // Если ячейка пуста
                return false;
            }
            if (table[index].key == key) { // Если ключ найден
                return true;
            }
            index = (index + 1) % table.size(); // Переходим к следующей ячейке
        } while (index != start_index); // Пока не вернемся в начальную ячейку
        // Используем именно do-while, чтобы хотя бы один раз проверить начальную ячейку

        return false; // Если не нашли ключ
    }

    void add(const Key& key, const Value& value) override { // Добавление элемента
        if (current_size >= table.size() / 2) { // Если таблица заполнена на 50% и более
            rehash(); // Перехешируем
        }

        size_t index = hash(key); // Вычисляем индекс
        while (table[index].isOccupied) { // Пока ячейка занята
            if (table[index].key == key) { // Если ключ найден
                table[index].value = value; // Обновляем значение
                return; // Завершаем функцию
            }
            index = (index + 1) % table.size(); // Переходим к следующей ячейке
        }

        table[index] = {key, value, true}; // Добавляем элемент
        ++current_size; // Увеличиваем количество элементов
    }

    void remove(const Key& key) override { // Удаление элемента
        size_t index = hash(key); // Вычисляем индекс
        size_t start_index = index; // Запоминаем начальный индекс

        do {
            if (!table[index].isOccupied) { // Если ячейка пуста
                throw std::runtime_error("Key not found"); // Выбрасываем исключение
            }
            if (table[index].key == key) { // Если ключ найден
                table[index].isOccupied = false; // Освобождаем ячейку
                --current_size; // Уменьшаем количество элементов
                return; // Завершаем функцию
            }
            index = (index + 1) % table.size(); // Переходим к следующей ячейке
        } while (index != start_index); // Пока не вернемся в начальную ячейку

        throw std::runtime_error("Key not found"); // Если не нашли ключ
    }

    Value& get(const Key& key) override { // Получение значения по ключу
        size_t index = hash(key); // Вычисляем индекс
        size_t start_index = index; // Запоминаем начальный индекс

        do { // Пока не найдем пустую ячейку или ячейку с нужным ключом
            if (!table[index].isOccupied) { // Если ячейка пуста
                throw std::runtime_error("Key not found"); // Выбрасываем исключение
            }
            if (table[index].key == key) { // Если ключ найден
                return table[index].value; // Возвращаем значение
            }
            index = (index + 1) % table.size(); // Переходим к следующей ячейке
        } while (index != start_index); // Пока не вернемся в начальную ячейку

        throw std::runtime_error("Key not found"); // Если не нашли ключ
    }

    const Value& get(const Key& key) const override { // То же самое, но теперь const
        size_t index = hash(key);
        size_t start_index = index;

        do {
            if (!table[index].isOccupied) {
                throw std::runtime_error("Key not found");
            }
            if (table[index].key == key) {
                return table[index].value;
            }
            index = (index + 1) % table.size();
        } while (index != start_index);

        throw std::runtime_error("Key not found");
    }

    Value& operator[](const Key& key) override { // Оператор доступа по ключу
        size_t index = hash(key); // Вычисляем индекс
        while (table[index].isOccupied) { // Пока ячейка занята
            if (table[index].key == key) {
                return table[index].value;
            }
            index = (index + 1) % table.size();
        }

        if (current_size >= table.size() / 2) { // Если таблица заполнена на 50% и более
            rehash(); // Перехешируем
            return (*this)[key]; // Рекурсивно вызываем оператор с новой таблицей
        }

        table[index] = {key, Value{}, true}; // Добавляем элемент ({} нужно для Value, чтобы создать пустой объект)
        ++current_size; // Увеличиваем количество элементов
        return table[index].value; // Возвращаем значение
    }

    const Value& operator[](const Key& key) const override {
        size_t index = hash(key); // Вычисляем индекс
        while (table[index].isOccupied) { // Пока ячейка занята
            if (table[index].key == key) {
                return table[index].value;
            }
            index = (index + 1) % table.size();
        }

        throw std::runtime_error("Key not found"); // Если не нашли ключ
    }

    auto begin() {
        return table.begin();
    }

    auto end() {
        return table.end();
    }

    auto begin() const {
        return table.begin();
    }

    auto end() const {
        return table.end();
    }
};

#endif //L3_DICTIONARY_H
