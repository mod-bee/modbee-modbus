#pragma once

#include "ModbusTypes.h"

#include <string.h>

// =============================================================================
// VALUES WIDER THAN A REGISTER
// =============================================================================
//
// A Modbus register is sixteen anonymous bits. A flow rate, a kilowatt-hour
// total, a position - none of them fit, so every one spans two registers or
// four, and NOTHING ON THE WIRE SAYS WHICH WAY ROUND.
//
// That is not a gap this library can close. What it can do is make the choice
// one explicit argument instead of an afternoon: name all four orderings the
// way vendor manuals name them, and ship a probe that reads a value four ways
// and shows the caller all four.

// How a multi-register value is laid out across registers.
//
// Named for the byte order of a 32-bit value whose bytes are A (most
// significant) through D (least). The names are the ones vendor manuals use,
// so a data sheet saying "CDAB" translates without thought.
enum class ModbusOrder : uint8_t {
    // A B | C D. Big-endian throughout, high word first. What the
    // specification implies and what most devices do.
    ABCD = 0,

    // C D | A B. Low word first, big-endian within each word. Extremely
    // common: Schneider, many drives, most of the "swapped words" settings in
    // SCADA packages.
    CDAB,

    // B A | D C. High word first, byte-swapped within each word. Rare, and
    // usually a sign that something is byte-swapping a buffer somewhere.
    BADC,

    // D C | B A. Fully reversed - little-endian throughout. What a device does
    // when it memcpy's a float straight out of a little-endian CPU.
    DCBA,
};

const char* mb_orderName(ModbusOrder order);

// Rearranges `length` wire bytes into value bytes, or back. The transform is
// its own inverse for all four orderings, so one function serves both
// directions - which also means a round trip cannot disagree with itself.
//
// `length` must be 4 or 8.
inline void mb_reorder(const uint8_t* in, uint8_t* out, size_t length, ModbusOrder order) {
    switch (order) {
        case ModbusOrder::ABCD:
            memcpy(out, in, length);
            break;

        case ModbusOrder::CDAB:
            // Reverse the order of the 16-bit words, keeping each word's own
            // bytes as they are.
            for (size_t i = 0; i < length; i += 2) {
                out[i]     = in[length - 2 - i];
                out[i + 1] = in[length - 1 - i];
            }
            break;

        case ModbusOrder::BADC:
            // Swap the two bytes inside each word, keeping the word order.
            for (size_t i = 0; i < length; i += 2) {
                out[i]     = in[i + 1];
                out[i + 1] = in[i];
            }
            break;

        case ModbusOrder::DCBA:
            for (size_t i = 0; i < length; i++) {
                out[i] = in[length - 1 - i];
            }
            break;
    }
}

// =============================================================================
// DECODING, FROM WIRE BYTES
// =============================================================================
//
// `wire` points at the big-endian register bytes as they arrived - two per
// register, in the order the response carried them.

inline uint32_t mb_toUInt32(const uint8_t* wire, ModbusOrder order) {
    uint8_t bytes[4];
    mb_reorder(wire, bytes, 4, order);
    return ((uint32_t)bytes[0] << 24) | ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8)  | (uint32_t)bytes[3];
}

inline int32_t mb_toInt32(const uint8_t* wire, ModbusOrder order) {
    return (int32_t)mb_toUInt32(wire, order);
}

inline float mb_toFloat32(const uint8_t* wire, ModbusOrder order) {
    // Through a byte copy rather than a pointer cast: casting a uint32_t* to a
    // float* and dereferencing it is undefined behaviour, and on a target with
    // strict alignment it is a fault rather than a wrong answer.
    const uint32_t bits = mb_toUInt32(wire, order);
    float value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

inline uint64_t mb_toUInt64(const uint8_t* wire, ModbusOrder order) {
    uint8_t bytes[8];
    mb_reorder(wire, bytes, 8, order);
    uint64_t value = 0;
    for (int i = 0; i < 8; i++) value = (value << 8) | bytes[i];
    return value;
}

inline int64_t mb_toInt64(const uint8_t* wire, ModbusOrder order) {
    return (int64_t)mb_toUInt64(wire, order);
}

inline double mb_toFloat64(const uint8_t* wire, ModbusOrder order) {
    const uint64_t bits = mb_toUInt64(wire, order);
    double value;
    memcpy(&value, &bits, sizeof value);
    return value;
}

// =============================================================================
// ENCODING, TO WIRE BYTES
// =============================================================================

inline void mb_fromUInt32(uint32_t value, uint8_t* wire, ModbusOrder order) {
    const uint8_t bytes[4] = {
        (uint8_t)(value >> 24), (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),  (uint8_t)value,
    };
    mb_reorder(bytes, wire, 4, order);
}

inline void mb_fromInt32(int32_t value, uint8_t* wire, ModbusOrder order) {
    mb_fromUInt32((uint32_t)value, wire, order);
}

inline void mb_fromFloat32(float value, uint8_t* wire, ModbusOrder order) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof bits);
    mb_fromUInt32(bits, wire, order);
}

inline void mb_fromUInt64(uint64_t value, uint8_t* wire, ModbusOrder order) {
    uint8_t bytes[8];
    for (int i = 7; i >= 0; i--) { bytes[i] = (uint8_t)value; value >>= 8; }
    mb_reorder(bytes, wire, 8, order);
}

inline void mb_fromInt64(int64_t value, uint8_t* wire, ModbusOrder order) {
    mb_fromUInt64((uint64_t)value, wire, order);
}

inline void mb_fromFloat64(double value, uint8_t* wire, ModbusOrder order) {
    uint64_t bits;
    memcpy(&bits, &value, sizeof bits);
    mb_fromUInt64(bits, wire, order);
}

// =============================================================================
// REGISTER-ARRAY FORMS
// =============================================================================
//
// The same, for callers holding registers rather than wire bytes.

inline void mb_registersToWire(const uint16_t* registers, size_t count, uint8_t* wire) {
    for (size_t i = 0; i < count; i++) mb_packBE16(wire + i * 2, registers[i]);
}

inline void mb_wireToRegisters(const uint8_t* wire, size_t count, uint16_t* registers) {
    for (size_t i = 0; i < count; i++) registers[i] = mb_unpackBE16(wire + i * 2);
}

inline float mb_registersToFloat32(const uint16_t* registers, ModbusOrder order) {
    uint8_t wire[4];
    mb_registersToWire(registers, 2, wire);
    return mb_toFloat32(wire, order);
}

inline int32_t mb_registersToInt32(const uint16_t* registers, ModbusOrder order) {
    uint8_t wire[4];
    mb_registersToWire(registers, 2, wire);
    return mb_toInt32(wire, order);
}

inline uint32_t mb_registersToUInt32(const uint16_t* registers, ModbusOrder order) {
    uint8_t wire[4];
    mb_registersToWire(registers, 2, wire);
    return mb_toUInt32(wire, order);
}

inline double mb_registersToFloat64(const uint16_t* registers, ModbusOrder order) {
    uint8_t wire[8];
    mb_registersToWire(registers, 4, wire);
    return mb_toFloat64(wire, order);
}

inline int64_t mb_registersToInt64(const uint16_t* registers, ModbusOrder order) {
    uint8_t wire[8];
    mb_registersToWire(registers, 4, wire);
    return mb_toInt64(wire, order);
}

inline void mb_float32ToRegisters(float value, uint16_t* registers, ModbusOrder order) {
    uint8_t wire[4];
    mb_fromFloat32(value, wire, order);
    mb_wireToRegisters(wire, 2, registers);
}

inline void mb_int32ToRegisters(int32_t value, uint16_t* registers, ModbusOrder order) {
    uint8_t wire[4];
    mb_fromInt32(value, wire, order);
    mb_wireToRegisters(wire, 2, registers);
}

inline void mb_uint32ToRegisters(uint32_t value, uint16_t* registers, ModbusOrder order) {
    uint8_t wire[4];
    mb_fromUInt32(value, wire, order);
    mb_wireToRegisters(wire, 2, registers);
}

inline void mb_float64ToRegisters(double value, uint16_t* registers, ModbusOrder order) {
    uint8_t wire[8];
    mb_fromFloat64(value, wire, order);
    mb_wireToRegisters(wire, 4, registers);
}

inline void mb_int64ToRegisters(int64_t value, uint16_t* registers, ModbusOrder order) {
    uint8_t wire[8];
    mb_fromInt64(value, wire, order);
    mb_wireToRegisters(wire, 4, registers);
}

// =============================================================================
// THE PROBE
// =============================================================================
//
// Every interpretation of the same registers, side by side.
//
// This is the diagnostic that turns "the flow reading is nonsense" from an
// afternoon into a minute. Read two registers, call this, and look for the
// interpretation that is a plausible flow rate; that is the device's ordering.
struct ModbusOrderProbe {
    float    asFloat32[4];    // Indexed by ModbusOrder
    int32_t  asInt32[4];
    uint32_t asUInt32[4];
};

inline ModbusOrderProbe mb_probeOrder(const uint16_t* registers) {
    ModbusOrderProbe probe;
    for (int i = 0; i < 4; i++) {
        const ModbusOrder order = (ModbusOrder)i;
        probe.asFloat32[i] = mb_registersToFloat32(registers, order);
        probe.asInt32[i]   = mb_registersToInt32(registers, order);
        probe.asUInt32[i]  = mb_registersToUInt32(registers, order);
    }
    return probe;
}

// =============================================================================
// TEXT IN REGISTERS
// =============================================================================
//
// Two characters per register, the first in the HIGH byte. The convention
// almost everything uses, and the one ModbusDataMap's string spans implement.

inline size_t mb_registersToText(const uint16_t* registers, size_t count,
                                 char* out, size_t capacity) {
    if (!out || capacity == 0) return 0;
    size_t at = 0;
    for (size_t i = 0; i < count && at + 1 < capacity; i++) {
        const char high = (char)(registers[i] >> 8);
        const char low  = (char)(registers[i] & 0xFF);
        if (high == '\0') break;
        out[at++] = high;
        if (at + 1 >= capacity || low == '\0') break;
        out[at++] = low;
    }
    out[at] = '\0';
    return at;
}

// Returns the registers used. Pads the last register with NUL where the text
// is an odd length, so nothing downstream reads a stray byte as a character.
inline size_t mb_textToRegisters(const char* text, uint16_t* registers, size_t capacity) {
    if (!text || !registers) return 0;
    const size_t length = strlen(text);
    size_t used = 0;
    for (size_t i = 0; i < length && used < capacity; i += 2) {
        const uint8_t high = (uint8_t)text[i];
        const uint8_t low  = (i + 1 < length) ? (uint8_t)text[i + 1] : 0;
        registers[used++] = (uint16_t)(((uint16_t)high << 8) | low);
    }
    return used;
}
