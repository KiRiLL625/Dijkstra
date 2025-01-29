#ifndef L3_DICTIONARY_H
#define L3_DICTIONARY_H

#include "IDictionary.h"
#include "ArraySequence.h"
#include <stdexcept>
#include <iostream>

template <typename Key, typename Value>
class Dictionary : public IDictionary<Key, Value> {
private:
    struct KeyValue {
        Key key;
        Value value;
        bool isOccupied = false;

        KeyValue() = default;
        KeyValue(const Key& key, const Value& value, bool isOccupied)
            : key(key), value(value), isOccupied(isOccupied) {}
    };

    ArraySequence<KeyValue>* table;
    size_t current_size;
    size_t capacity_;

    size_t hash(const Key& key) const {
        //std::cout << "Hash function" << std::endl;
        //std::cout << "Length: " << capacity_ << std::endl;
        return std::hash<Key>{}(key) % capacity_;
    }

    void rehash() {
        size_t old_capacity = capacity_;
        size_t new_capacity = capacity_ * 2;
        auto* new_table = new ArraySequence<KeyValue>(new_capacity);

        for (size_t i = 0; i < new_capacity; ++i) {
            new_table->append({});
        }

        for (size_t i = 0; i < capacity_; ++i) {
            if ((*table)[i].isOccupied) {
                size_t new_index = std::hash<Key>{}((*table)[i].key) % new_capacity;

                while ((*new_table)[new_index].isOccupied) {
                    new_index = (new_index + 1) % new_capacity;
                }

                (*new_table)[new_index] = (*table)[i];
            }
        }

        //std::cout << "Rehashing: Old capacity: " << old_capacity
        //          << " New capacity: " << new_capacity << std::endl;

        delete table;  // Удаляем старый массив
        table = new_table;
        capacity_ = new_capacity;
    }

public:
    Dictionary() : current_size(0), capacity_(16) {
        table = new ArraySequence<KeyValue>(capacity_);
        for (size_t i = 0; i < capacity_; ++i) {
            table->append({});
        }
    }

    explicit Dictionary(size_t initial_capacity)
        : current_size(0), capacity_(initial_capacity) {
        table = new ArraySequence<KeyValue>(capacity_);
        for (size_t i = 0; i < capacity_; ++i) {
            table->append({});
        }
    }

    ~Dictionary() {
        delete table;
    }

    size_t count() const override {
        return current_size;
    }

    size_t capacity() const override {
        return capacity_;
    }

    bool contains_key(const Key& key) const override {
        size_t index = hash(key);
        size_t start_index = index;

        do {
            if (!(*table)[index].isOccupied) {
                return false;
            }
            if ((*table)[index].key == key) {
                return true;
            }
            index = (index + 1) % capacity_;
        } while (index != start_index);

        return false;
    }

    void add(const Key& key, const Value& value) override {
        if (current_size >= capacity_ / 2) {
            //std::cout << "Rehashing triggered. Current size: " << current_size
             //         << " Capacity: " << capacity_ << std::endl;
            rehash();
        }

        size_t index = hash(key);
        while ((*table)[index].isOccupied) {
            if ((*table)[index].key == key) {
                (*table)[index].value = value;
                return;
            }
            index = (index + 1) % capacity_;
        }

        (*table)[index] = {key, value, true};
        ++current_size;
        //std::cout << "Added key: " << key << " at index: " << index
        //          << " after " << probeCount << " probes" << std::endl;
    }

    void remove(const Key& key) override {
        size_t index = hash(key);
        size_t start_index = index;

        do {
            if (!(*table)[index].isOccupied) {
                throw std::runtime_error("Key not found");
            }
            if ((*table)[index].key == key) {
                (*table)[index].isOccupied = false;
                --current_size;
                return;
            }
            index = (index + 1) % capacity_;
        } while (index != start_index);

        throw std::runtime_error("Key not found");
    }

    Value& get(const Key& key) override {
        size_t index = hash(key);
        size_t start_index = index;

        do {
            if (!(*table)[index].isOccupied) {
                throw std::runtime_error("Key not found");
            }
            if ((*table)[index].key == key) {
                return (*table)[index].value;
            }
            index = (index + 1) % capacity_;
        } while (index != start_index);

        throw std::runtime_error("Key not found");
    }

    const Value& get(const Key& key) const override {
        size_t index = hash(key);
        size_t start_index = index;

        do {
            if (!(*table)[index].isOccupied) {
                throw std::runtime_error("Key not found");
            }
            if ((*table)[index].key == key) {
                return (*table)[index].value;
            }
            index = (index + 1) % capacity_;
        } while (index != start_index);

        throw std::runtime_error("Key not found");
    }

    Value& operator[](const Key& key) override {
        size_t index = hash(key);
        while ((*table)[index].isOccupied) {
            if ((*table)[index].key == key) {
                return (*table)[index].value;
            }
            index = (index + 1) % capacity_;
        }

        if (current_size >= capacity_ / 2) {
            rehash();
            index = hash(key);
        }

        (*table)[index] = {key, Value{}, true};
        ++current_size;
        return (*table)[index].value;
    }

    void clear() override {
        for (size_t i = 0; i < capacity_; ++i) {
            (*table)[i].isOccupied = false;
        }
        current_size = 0;
    }

    auto begin() {
        return table->begin();
    }

    auto end() {
        return table->end();
    }

    auto begin() const {
        return table->begin();
    }

    auto end() const {
        return table->end();
    }

    ArraySequence<Key> keys() const {
        ArraySequence<Key> keyList;
        for (const auto& entry : *table) {
            if (entry.isOccupied) {
                keyList.append(entry.key);
            }
        }
        return keyList;
    }
};

#endif // L3_DICTIONARY_H
