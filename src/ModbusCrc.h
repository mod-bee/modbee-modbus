#pragma once

#include "ModbusPlatform.h"

// Checksums for the two serial ADUs.
//
// Pure functions over buffers: no state, no allocation, no clock. The codecs
// own where these bytes go; this file only says what they are.

// --- CRC-16/MODBUS ---
//
// Reflected polynomial 0xA001 (the reverse of 0x8005), initial value 0xFFFF,
// no final XOR, input and output reflected.
//
// The one little-endian field on the whole Modbus wire: the CRC is appended
// LOW byte first, where every 16-bit field inside a PDU is big-endian. Getting
// this backwards produces frames that every device rejects and no test that
// checks only its own output will catch, so append() exists and is what the
// codecs use.
uint16_t mb_crc16(const uint8_t* data, size_t length);

// Continues a CRC over another run of bytes, for a codec that hashes a header
// and a payload held in different buffers.
uint16_t mb_crc16Update(uint16_t crc, const uint8_t* data, size_t length);

// Writes a CRC into two bytes, low byte first, as the wire carries it.
inline void mb_crc16Append(uint8_t* out, uint16_t crc) {
    out[0] = (uint8_t)(crc & 0xFF);
    out[1] = (uint8_t)(crc >> 8);
}

// Reads a CRC from two wire bytes, low byte first.
inline uint16_t mb_crc16Read(const uint8_t* in) {
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

// Whether a whole RTU ADU, its trailing CRC included, checks out.
inline bool mb_crc16Check(const uint8_t* adu, size_t length) {
    if (length < 3) return false;   // addr + at least a function code + CRC
    return mb_crc16(adu, length - 2) == mb_crc16Read(adu + length - 2);
}

// --- LRC, for Modbus ASCII ---
//
// The two's complement of the sum of the bytes, taken over the binary values
// of address and PDU, before hex encoding. One byte, carried as two hex
// characters ahead of the CRLF.
uint8_t mb_lrc(const uint8_t* data, size_t length);

// --- Hex, for Modbus ASCII ---
//
// Transmitted uppercase; accepted either case, because the specification says
// uppercase and devices in the field do not always agree.

inline char mb_hexDigit(uint8_t nibble) {
    return (char)(nibble < 10 ? '0' + nibble : 'A' + (nibble - 10));
}

// The value of one hex character, or -1 if it is not one.
inline int mb_hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
