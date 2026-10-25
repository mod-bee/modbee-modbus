#include "ModbusCrc.h"
#include "ModbusNoExcept.h"

namespace {

// Bitwise rather than table-driven. A 512-byte table would buy roughly a factor
// of eight on a CRC that costs microseconds on a link measured in milliseconds,
// and 512 bytes of flash is a real cost on the small end of the target range.
// If profiling ever says otherwise the table goes here and nothing else moves.
inline uint16_t crcStep(uint16_t crc, uint8_t byte) {
    crc = (uint16_t)(crc ^ byte);
    for (uint8_t bit = 0; bit < 8; bit++) {
        if (crc & 1u) {
            crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        } else {
            crc = (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

} // namespace

uint16_t mb_crc16Update(uint16_t crc, const uint8_t* data, size_t length) {
    if (!data) return crc;
    for (size_t i = 0; i < length; i++) {
        crc = crcStep(crc, data[i]);
    }
    return crc;
}

uint16_t mb_crc16(const uint8_t* data, size_t length) {
    return mb_crc16Update(0xFFFFu, data, length);
}

uint8_t mb_lrc(const uint8_t* data, size_t length) {
    if (!data) return 0;
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    // Two's complement. Written as a negation of the sum rather than
    // (0x100 - sum) so the zero case needs no special handling.
    return (uint8_t)(-(int8_t)sum);
}
