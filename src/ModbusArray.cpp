#include "ModbusArray.h"
#include "ModbusNoExcept.h"

bool mb_arrayGrow(void** data, uint16_t* capacity, uint16_t count, size_t needed, size_t size) {
    if (needed > 0xFFFF) return false;
    size_t grown = *capacity ? (size_t)*capacity * 2 : 4;
    if (grown < needed) grown = needed;
    if (grown > 0xFFFF) grown = 0xFFFF;
    void* fresh = ::operator new(grown * size, std::nothrow);
    if (!fresh) return false;
    if (count) memcpy(fresh, *data, (size_t)count * size);
    ::operator delete(*data);
    *data = fresh;
    *capacity = (uint16_t)grown;
    return true;
}

void mb_arrayFree(void* data) {
    ::operator delete(data);
}
