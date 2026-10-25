#pragma once

// Self-contained so codecs, transports and tests can pull in the wire
// constants without dragging in the whole library.
#include "ModbusPlatform.h"

// =============================================================================
// BYTE ORDER
// =============================================================================

// Everything 16-bit in a Modbus PDU is big-endian, without exception. The one
// little-endian field on the whole wire is the RTU CRC, which lives in the ADU
// and is written by the codec, not here.

inline void mb_packBE16(uint8_t* out, uint16_t value) {
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)(value & 0xFF);
}

inline uint16_t mb_unpackBE16(const uint8_t* in) {
    return (uint16_t)(((uint16_t)in[0] << 8) | in[1]);
}

// =============================================================================
// FUNCTION CODES
// =============================================================================

#define MB_FC_READ_COILS                0x01
#define MB_FC_READ_DISCRETE_INPUTS      0x02
#define MB_FC_READ_HOLDING_REGISTERS    0x03
#define MB_FC_READ_INPUT_REGISTERS      0x04
#define MB_FC_WRITE_SINGLE_COIL         0x05
#define MB_FC_WRITE_SINGLE_REGISTER     0x06
#define MB_FC_READ_EXCEPTION_STATUS     0x07   // Serial line only
#define MB_FC_DIAGNOSTICS               0x08   // Serial line only
#define MB_FC_GET_COMM_EVENT_COUNTER    0x0B   // Serial line only
#define MB_FC_GET_COMM_EVENT_LOG        0x0C   // Serial line only
#define MB_FC_WRITE_MULTIPLE_COILS      0x0F
#define MB_FC_WRITE_MULTIPLE_REGISTERS  0x10
#define MB_FC_REPORT_SERVER_ID          0x11   // Serial line only
#define MB_FC_READ_FILE_RECORD          0x14
#define MB_FC_WRITE_FILE_RECORD         0x15
#define MB_FC_MASK_WRITE_REGISTER       0x16
#define MB_FC_READ_WRITE_REGISTERS      0x17
#define MB_FC_READ_FIFO_QUEUE           0x18
#define MB_FC_ENCAPSULATED_INTERFACE    0x2B   // With MEI type below

// MEI types carried by 0x2B.
#define MB_MEI_CANOPEN                  0x0D   // Not implemented; recognised so
                                               // it is refused as a function
                                               // rather than misread
#define MB_MEI_READ_DEVICE_ID           0x0E

// The exception bit. A response function code with this set carries one
// exception code and nothing else.
#define MB_FC_EXCEPTION_MASK            0x80

inline bool mb_isExceptionFunction(uint8_t function) {
    return (function & MB_FC_EXCEPTION_MASK) != 0;
}

inline uint8_t mb_exceptionFunction(uint8_t function) {
    return (uint8_t)(function | MB_FC_EXCEPTION_MASK);
}

inline uint8_t mb_baseFunction(uint8_t function) {
    return (uint8_t)(function & 0x7F);
}

// True for the function codes whose response carries read data.
inline bool mb_isReadFunction(uint8_t function) {
    switch (mb_baseFunction(function)) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
        case MB_FC_READ_WRITE_REGISTERS:
            return true;
        default:
            return false;
    }
}

// True for the bit-oriented reads, whose responses carry a byte count rather
// than a bit count. The requester is the only one who knows how many bits it
// asked for, which is why a response is always decoded against its request.
inline bool mb_isBitReadFunction(uint8_t function) {
    const uint8_t base = mb_baseFunction(function);
    return base == MB_FC_READ_COILS || base == MB_FC_READ_DISCRETE_INPUTS;
}

// True for the function codes that change server state. A broadcast may only
// carry one of these: there is no response to a broadcast, so a broadcast read
// would ask a question nobody can answer.
inline bool mb_isWriteFunction(uint8_t function) {
    switch (mb_baseFunction(function)) {
        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
        case MB_FC_MASK_WRITE_REGISTER:
        case MB_FC_WRITE_FILE_RECORD:
            return true;
        default:
            return false;
    }
}

// Function codes defined only for serial line. A TCP server should refuse them
// with IllegalFunction rather than answer, because their semantics are tied to
// a shared bus and a single device behind one address.
inline bool mb_isSerialOnlyFunction(uint8_t function) {
    switch (mb_baseFunction(function)) {
        case MB_FC_READ_EXCEPTION_STATUS:
        case MB_FC_DIAGNOSTICS:
        case MB_FC_GET_COMM_EVENT_COUNTER:
        case MB_FC_GET_COMM_EVENT_LOG:
        case MB_FC_REPORT_SERVER_ID:
            return true;
        default:
            return false;
    }
}

// Short name, for logging and for test output that has to be read by a human.
const char* mb_functionName(uint8_t function);

// =============================================================================
// EXCEPTION CODES
// =============================================================================

// Not a Modbus code. Zero is not a legal exception, so it stands for "no
// exception" wherever a function returns one, which lets decode and validate
// return the exception the server should reply with rather than a bool the
// caller then has to translate.
#define MB_EX_NONE                      0x00

#define MB_EX_ILLEGAL_FUNCTION          0x01
#define MB_EX_ILLEGAL_DATA_ADDRESS      0x02
#define MB_EX_ILLEGAL_DATA_VALUE        0x03
#define MB_EX_SERVER_DEVICE_FAILURE     0x04

// Accepted, and it will take a while. The client must POLL for completion, not
// retry. Mishandled, this reads as a hang; treated as a failure, a long-running
// command is reported failed while it is still running.
#define MB_EX_ACKNOWLEDGE               0x05

// Busy with a long-duration command. The client should RETRY later. Distinct
// from Acknowledge, and the difference is the whole of what the client does
// next.
#define MB_EX_SERVER_DEVICE_BUSY        0x06

// Negative acknowledge. Legacy, seen on older devices answering 0x14/0x15.
#define MB_EX_NEGATIVE_ACKNOWLEDGE      0x07

// Extended file area failed a consistency check. Used with 0x14 / 0x15.
#define MB_EX_MEMORY_PARITY_ERROR       0x08

// The gateway could not allocate a path from its input port to its output port.
#define MB_EX_GATEWAY_PATH_UNAVAILABLE  0x0A

// No response from the target device behind a gateway. This is the one our own
// gateway mode emits when a downstream serial server times out, and it is what
// tells a TCP client "the network is fine, the device is not there".
#define MB_EX_GATEWAY_TARGET_FAILED     0x0B

const char* mb_exceptionName(uint8_t exceptionCode);

// =============================================================================
// PROTOCOL LIMITS
// =============================================================================

// The PDU is 253 bytes, everywhere and always. Every ADU is that plus its own
// framing, and every quantity limit below is derived from it.
#define MB_MAX_PDU              253

#define MB_RTU_ADU_MAX          256    // addr(1) + PDU + CRC(2)
#define MB_TCP_ADU_MAX          260    // MBAP(7) + PDU
#define MB_ASCII_ADU_MAX        513    // ':' + 2*(addr+PDU+LRC) + CRLF

// Quantity limits, per the specification and confirmed against libmodbus
// 3.1.11. Each is what fits in a PDU once that
// function's own header is accounted for.
#define MB_MAX_READ_BITS        2000   // 0x01, 0x02
#define MB_MAX_WRITE_BITS       1968   // 0x0F
#define MB_MAX_READ_REGISTERS   125    // 0x03, 0x04
#define MB_MAX_WRITE_REGISTERS  123    // 0x10

// 0x17 carries a longer request header, so its write side is capped lower than
// 0x10's. FC(1) + readAddr(2) + readQty(2) + writeAddr(2) + writeQty(2) +
// byteCount(1) = 10, leaving 243 bytes for data: 121 registers, not 123.
#define MB_MAX_RW_READ_REGISTERS   125
#define MB_MAX_RW_WRITE_REGISTERS  121

// A byte count is one byte, so no read response may declare more than 255
// bytes. Every quantity limit above is already inside this; it is asserted
// separately because a byte count that wrapped would be read as a short
// response rather than as an error.
#define MB_MAX_BYTE_COUNT       255

// =============================================================================
// ADDRESSING
// =============================================================================

// Serial line addressing. 0 is broadcast, 1..247 are individually addressable,
// and 248..255 are reserved.
//
// Our client refuses to ADDRESS 248..255, and our server answers only on
// 1..247. Our receive path does not treat 248..255 as a framing error, because
// real devices use them: diagslave accepts 1..255.
#define MB_ADDRESS_BROADCAST    0
#define MB_ADDRESS_MIN          1
#define MB_ADDRESS_MAX          247
#define MB_ADDRESS_RESERVED_MIN 248

// TCP unit identifier. 255 conventionally means "the device itself" rather
// than something behind it, and 0 is widely accepted as the same. A TCP server
// that is not a gateway answers whatever unit it is asked for on its own
// socket, so neither is special to us except in gateway mode.
#define MB_UNIT_SELF            255

// Returned by silenceFor() for a unit that has never answered.
//
// Deliberately not zero and not a huge elapsed time: "never heard from" is a
// commissioning problem - a wrong unit id, a device never fitted - and "heard
// from once, then silent" is a running fault. An application that cannot tell
// them apart sends somebody to the wrong end of the plant.
#define MB_NEVER_HEARD          0xFFFFFFFFu

inline bool mb_isAddressableUnit(uint8_t unit) {
    return unit >= MB_ADDRESS_MIN && unit <= MB_ADDRESS_MAX;
}

// Whether a start address and quantity stay inside the 16-bit address space.
//
// A block is walked with 16-bit arithmetic, so one that runs past 65535
// reappears at address 0 rather than ending. The addresses it would then reach
// are real, mapped and completely unrelated, so the caller would be handed
// someone else's registers with no way to tell.
inline bool mb_isValidAddressRange(uint16_t startAddr, uint16_t quantity) {
    return (uint32_t)startAddr + (uint32_t)quantity <= 0x10000u;
}

// =============================================================================
// DATA TABLES
// =============================================================================

// The four Modbus data tables. Named for what they are rather than for their
// 0x/1x/3x/4x reference numbers, because those numbers are a convention of
// documentation and not of the wire.
enum class ModbusTable : uint8_t {
    Coil = 0,          // Read/write bits      - FC 01, 05, 0F
    DiscreteInput,     // Read-only bits       - FC 02
    HoldingRegister,   // Read/write registers - FC 03, 06, 10, 16, 17
    InputRegister,     // Read-only registers  - FC 04
};


// Parity, and with it the number of stop bits.
//
// Modbus serial defaults to EVEN parity. Where parity is unavailable the
// specification requires TWO stop bits, precisely so the character time stays
// at eleven bits and the t1.5/t3.5 figures derived from it do not change.
// Choosing None here therefore selects 8N2, not 8N1 - and a device configured
// for 8N1 at the other end is off-spec, which is worth knowing before spending
// an afternoon on it.
//
// Declared HERE, with the other wire-level types, rather than beside the host
// serial transport that consumes it. It describes a line, not a platform, and
// a binding that offers parity as a setting has to be able to name the type on
// a board as well - where the host transport does not compile at all.
enum class ModbusParity : uint8_t {
    Even = 0,   // 8E1 - the Modbus default
    Odd,        // 8O1
    None,       // 8N2, per the specification
    None8N1,    // 8N1 - off-spec, but common. Provided because devices exist.
};

inline bool mb_tableIsBits(ModbusTable table) {
    return table == ModbusTable::Coil || table == ModbusTable::DiscreteInput;
}

// Whether a client can write this table. There is no write function code for
// either input table; that is a property of the protocol, not a permission.
inline bool mb_tableIsWritable(ModbusTable table) {
    return table == ModbusTable::Coil || table == ModbusTable::HoldingRegister;
}

const char* mb_tableName(ModbusTable table);

// The table a function code operates on. Only meaningful for the data-access
// codes; anything else answers HoldingRegister and the caller must not ask.
ModbusTable mb_tableForFunction(uint8_t function);

// =============================================================================
// BIT PACKING
// =============================================================================

// Modbus packs bits LSB-first within each byte: the first coil of the request
// is bit 0 of the first byte. Unused high bits of the last byte are zero.

inline uint16_t mb_packedByteCount(uint16_t bitCount) {
    return (uint16_t)((bitCount + 7) / 8);
}

inline bool mb_getPackedBit(const uint8_t* packed, uint16_t index) {
    return (packed[index / 8] & (uint8_t)(1u << (index % 8))) != 0;
}

inline void mb_setPackedBit(uint8_t* packed, uint16_t index, bool value) {
    const uint8_t mask = (uint8_t)(1u << (index % 8));
    if (value) packed[index / 8] |= mask;
    else       packed[index / 8] = (uint8_t)(packed[index / 8] & ~mask);
}

// The two values a single-coil write may carry. Anything else is an illegal
// data value, which is a real check and not pedantry: a device that accepts
// 0x0001 as ON will read a neighbour's 0xFF00 as OFF.
#define MB_COIL_ON              0xFF00
#define MB_COIL_OFF             0x0000

// =============================================================================
// ELEMENT STRIDE
// =============================================================================

// One element of a caller's array. A stride of 0 means packed, which is what a
// plain C array is; anything else is the gap the caller's own layout puts
// between elements. This is what lets a response be unpacked straight into a
// PLC's located-variable table without an intermediate copy.
template <typename T>
inline T* mb_elemAt(void* base, uint16_t index, uint16_t stride) {
    return stride == 0 ? static_cast<T*>(base) + index
                       : reinterpret_cast<T*>(static_cast<uint8_t*>(base) +
                                              (size_t)index * stride);
}

template <typename T>
inline const T* mb_elemAt(const void* base, uint16_t index, uint16_t stride) {
    return stride == 0 ? static_cast<const T*>(base) + index
                       : reinterpret_cast<const T*>(static_cast<const uint8_t*>(base) +
                                                    (size_t)index * stride);
}
