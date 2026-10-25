#pragma once

// =============================================================================
// ModbusArray - the library's one container
// =============================================================================
//
// A list in one block of memory. It grows only when something is added past
// its capacity - which the library arranges to happen while a node is being
// set up (a binding, a watch, a route), or reserves up front for the tables
// that fill while it runs - so a running node does not allocate.
//
// Why not std::vector: each element type instantiates its own grow, copy and
// insert code, and that code throws, which keeps the exception tables in the
// firmware. Here elements are moved with memmove (everything the library
// stores is a plain record, its handlers included), the growing is one
// function shared by every element type, and nothing throws: an allocation
// that fails is reported to the caller as false.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <new>
#include <utility>

// Grows *data to hold at least `needed` elements of `size` bytes, keeping the
// first `count`. Doubles, from 4, so a list built one entry at a time is
// copied a handful of times, not once per entry. False if memory ran out;
// the list is then unchanged.
bool mb_arrayGrow(void** data, uint16_t* capacity, uint16_t count, size_t needed, size_t size);
void mb_arrayFree(void* data);

template <typename T>
class ModbusArray {
public:
    ModbusArray() noexcept {}
    ~ModbusArray() {
        clear();
        mb_arrayFree(_items);
    }
    ModbusArray(const ModbusArray&) = delete;
    ModbusArray& operator=(const ModbusArray&) = delete;

    // Hands the elements over whole: nothing copied, nothing freed.
    void swap(ModbusArray& other) noexcept {
        T* items = _items;
        _items = other._items;
        other._items = items;
        uint16_t n = _count;
        _count = other._count;
        other._count = n;
        n = _capacity;
        _capacity = other._capacity;
        other._capacity = n;
    }

    size_t size() const { return _count; }
    size_t capacity() const { return _capacity; }
    bool empty() const { return _count == 0; }

    T& operator[](size_t i) { return _items[i]; }
    const T& operator[](size_t i) const { return _items[i]; }
    T& back() { return _items[_count - 1]; }
    T* begin() { return _items; }
    T* end() { return _items + _count; }
    const T* begin() const { return _items; }
    const T* end() const { return _items + _count; }

    // Room for `needed` elements in all, allocated now.
    bool reserve(size_t needed) {
        return needed <= _capacity ||
               mb_arrayGrow((void**)&_items, &_capacity, _count, needed, sizeof(T));
    }

    bool push(const T& item) { return insert(_count, item); }

    bool insert(size_t at, const T& item) {
        if (_count == 0xFFFF || !reserve((size_t)_count + 1)) return false;
        memmove((void*)(_items + at + 1), (const void*)(_items + at), (_count - at) * sizeof(T));
        new ((void*)(_items + at)) T(item);
        _count++;
        return true;
    }

    void erase(size_t at, size_t n = 1) {
        for (size_t i = at; i < at + n; i++) _items[i].~T();
        memmove((void*)(_items + at), (const void*)(_items + at + n),
                (_count - at - n) * sizeof(T));
        _count = (uint16_t)(_count - n);
    }

    // Empties the list and keeps its memory.
    void clear() {
        for (size_t i = 0; i < _count; i++) _items[i].~T();
        _count = 0;
    }

private:
    T* _items = nullptr;
    uint16_t _count = 0;
    uint16_t _capacity = 0;
};
