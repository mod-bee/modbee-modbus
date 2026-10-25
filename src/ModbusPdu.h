#pragma once

#include "ModbusTypes.h"

// =============================================================================
// THE MODBUS PDU
// =============================================================================
//
// Pure functions over buffers. No I/O, no clock, no allocation, no state.
// Everything here is decided by the bytes in front of it, which is what makes
// this layer exhaustively testable against the specification's own worked
// examples and fuzzable in isolation.
//
// A PDU is [function code][function-specific data], 253 bytes at most. It
// carries no address and no checksum: those belong to the ADU, and which ADU
// is the codec's business, not this file's.
//
// --- Responses are decoded against their request ---
//
// A Modbus response carries no start address, and a bit-read response carries
// a byte count rather than a bit count. Only the client knows what it asked
// for.
//
// So decodeResponse() takes the request it answers. Everything a response does
// not say about itself is answered from the request that caused it, and there
// is nowhere else for that information to come from.
//
// --- Lifetime ---
//
// A decoded request or response holds POINTERS INTO THE BUFFER IT WAS DECODED
// FROM. Nothing is copied. The view is valid only while that buffer is, which
// is the whole of one receive cycle. Anything that must outlive it is copied
// by the caller.

// Returned by decodeRequest() when the PDU cannot be parsed at all - too short
// to determine what was even being asked. Not a Modbus exception code and
// never transmitted: on a serial line the correct answer to an unintelligible
// request is silence, and the server engine needs to be able to tell that case
// from one it should reply to.
#define MB_EX_DECODE_FAILED     0xFF

// =============================================================================
// REQUEST
// =============================================================================

// A decoded request, or one being built. Which fields carry meaning depends on
// the function code; the rest are zero. Named rather than overlaid, because a
// union here would save twelve bytes and cost every reader of this file.
struct ModbusRequest {
    uint8_t  function = 0;

    // Read functions, and the read half of 0x17. Also the target address of
    // every write, and the FIFO pointer of 0x18.
    uint16_t address = 0;
    uint16_t quantity = 0;

    // 0x17 only: the write half.
    uint16_t writeAddress = 0;
    uint16_t writeQuantity = 0;

    // 0x05 and 0x06: the value being written. For 0x05 this is MB_COIL_ON or
    // MB_COIL_OFF, not 1 or 0 - a device that accepts 0x0001 as ON will read a
    // neighbour's 0xFF00 as OFF.
    uint16_t value = 0;

    // 0x16 only.
    uint16_t andMask = 0;
    uint16_t orMask = 0;

    // 0x08 only. Sub-function 0x0000 is Return Query Data, the standard link
    // test, and the only one every device is expected to answer.
    uint16_t subFunction = 0;

    // 0x2B only. meiType selects what is encapsulated; only MB_MEI_READ_DEVICE_ID
    // is implemented, and MB_MEI_CANOPEN is recognised so it can be refused as
    // a function rather than misread as one.
    uint8_t  meiType = 0;
    uint8_t  readDeviceIdCode = 0;   // 01 basic, 02 regular, 03 extended, 04 one object
    uint8_t  objectId = 0;

    // Payload, excluding any byte-count byte, which is derived rather than
    // carried: a byte count the sender supplied and the parser trusted is how
    // a request reads past its own PDU.
    //
    //   0x0F  packed coil bits, LSB first
    //   0x10  register values, big-endian, two bytes each
    //   0x17  the registers to write
    //   0x08  sub-function data
    //   0x14  the read sub-requests
    //   0x15  the write sub-requests
    const uint8_t* data = nullptr;
    uint16_t dataLen = 0;

    // Registers or coils this request writes, derived from the function code.
    // 0 for a pure read.
    uint16_t writeCount() const;

    // The address the write half of this request targets. Equal to `address`
    // for every write except 0x17, which has two addresses.
    uint16_t writeTarget() const {
        return function == MB_FC_READ_WRITE_REGISTERS ? writeAddress : address;
    }
};

// Bytes this request's RESPONSE will occupy, at most, PDU only.
//
// Used to size a timeout against what the medium physically needs: a one
// register read and a 125 register read are the same request but very
// different amounts of airtime, and judging both against one figure declares
// the second failed while it is still arriving.
//
// An upper bound, because a device may answer with an exception instead - which
// is shorter, and being generous costs only a slow failure where being tight
// costs a healthy device declared dead.
uint16_t mb_expectedResponseBytes(const ModbusRequest& request);

// =============================================================================
// RESPONSE
// =============================================================================

// A decoded response. `quantity` is what the client asked for, carried through
// from the request, because for a bit read the wire does not say.
struct ModbusResponse {
    uint8_t  function = 0;

    // An exception response carries nothing else. Checked before any other
    // field is read, because noticing it one step too late retires the read as
    // though it had succeeded.
    bool     isException = false;
    uint8_t  exceptionCode = MB_EX_NONE;

    // Read data, after the byte count. Bits are packed LSB-first; registers
    // are big-endian pairs.
    const uint8_t* data = nullptr;
    uint16_t dataLen = 0;

    // Items `data` represents - bits or registers, per the function code.
    uint16_t quantity = 0;

    // Echoed by a write response, and checked against the request: a server
    // that echoes a different address or value than it was given has done
    // something other than what was asked.
    uint16_t address = 0;
    uint16_t value = 0;
    uint16_t andMask = 0;
    uint16_t orMask = 0;
};

// Why a response could not be decoded. Distinguished because the client does
// different things with each: a short response may still be arriving, a wrong
// function code is a device fault, and a malformed one is a bug somewhere.
enum class ModbusDecode : uint8_t {
    Ok = 0,
    Short,           // Fewer bytes than this function's response needs
    Malformed,       // Self-inconsistent: a byte count that does not fit
    WrongFunction,   // Answers a different function than the request asked
    Mismatch,        // Echoes an address, value or count the request did not send
};

const char* mb_decodeName(ModbusDecode result);

// =============================================================================
// ENCODING
// =============================================================================

namespace mbpdu {

// --- Requests (client side) ---
//
// Every encoder below shares one contract:
//
//   Returns the number of bytes written, or 0 if the input is not encodable or
//   will not fit. The capacity is always passed, because a builder that cannot
//   see the buffer it is writing into is how a batched write runs off the end
//   of a frame.
//
//   NOTHING IS EVER WRITTEN PAST `capacity`. That is the memory-safety
//   guarantee and it is fuzzed.
//
//   On a refusal the first `capacity` bytes are UNSPECIFIED: an encoder writes
//   as it goes and discovers it has run out at the end, rather than sizing
//   first. A caller must therefore act on the return value and not on the
//   buffer. Sizing first would mean a second table of per-function lengths
//   that could drift from the encoder, and one source of truth is worth more
//   here than a tidy buffer on a path the caller is required to check anyway.

// Encodes `req` into `out`. See the contract above.
size_t encodeRequest(uint8_t* out, size_t capacity, const ModbusRequest& req);

// The PDU length `req` would encode to, without writing anything. 0 when it is
// not encodable. Used to decide whether an operation fits before committing to
// it.
size_t requestSize(const ModbusRequest& req);

// --- Responses (server side) ---

// A read response: [FC][byteCount][payload]. `byteCount` is written from
// `payloadLen`, which must be 1..255.
size_t encodeReadResponse(uint8_t* out, size_t capacity, uint8_t function,
                          const uint8_t* payload, size_t payloadLen);

// The echo a write response carries. 0x05 and 0x06 echo address and value;
// 0x0F and 0x10 echo address and quantity; 0x16 echoes address and both masks.
// Built from the request, because an echo is by definition what was asked for.
size_t encodeEchoResponse(uint8_t* out, size_t capacity, const ModbusRequest& req);

// [FC][body]. For the responses whose shape is neither a read nor an echo:
// 0x07, 0x08, 0x0B, 0x0C, 0x11, 0x14, 0x15, 0x18, 0x2B.
size_t encodeRawResponse(uint8_t* out, size_t capacity, uint8_t function,
                         const uint8_t* body, size_t bodyLen);

// [FC|0x80][exceptionCode]. Two bytes, and no address.
//
// The absence is deliberate and is worth knowing: an exception says only WHAT
// went wrong, never WHERE. Which request it belongs to is settled by the
// transaction identifier on TCP, and on serial by the fact that only one
// request may be outstanding at a time.
size_t encodeException(uint8_t* out, size_t capacity, uint8_t function,
                       uint8_t exceptionCode);

// =============================================================================
// DECODING
// =============================================================================

// Decodes a request PDU.
//
// Returns MB_EX_NONE when `out` is filled in and the request is well formed;
// a Modbus exception code when the request is intelligible but wrong, which is
// exactly what the server should reply with; or MB_EX_DECODE_FAILED when it
// cannot be parsed at all and the only correct answer is silence.
//
// Structure only. Whether the addresses exist is the data map's question, not
// this one's.
uint8_t decodeRequest(const uint8_t* pdu, size_t length, ModbusRequest& out);

// The quantity and value limits, separately from decoding, so a client can
// refuse its own bad request before anything reaches the wire rather than
// waiting a round trip to be told. decodeRequest() applies these too.
uint8_t validateRequest(const ModbusRequest& req);

// Decodes a response PDU against the request it answers.
//
// The request is not optional. A Modbus response carries no address, and a bit
// read carries a byte count rather than a bit count, so what the client asked
// for is the only source for either.
ModbusDecode decodeResponse(const uint8_t* pdu, size_t length,
                            const ModbusRequest& req, ModbusResponse& out);

// =============================================================================
// CONVENIENCE BUILDERS
// =============================================================================
//
// Named constructors, so a call site says what it is asking for rather than
// which fields it happens to fill in.

ModbusRequest readCoils(uint16_t address, uint16_t quantity);
ModbusRequest readDiscreteInputs(uint16_t address, uint16_t quantity);
ModbusRequest readHoldingRegisters(uint16_t address, uint16_t quantity);
ModbusRequest readInputRegisters(uint16_t address, uint16_t quantity);

ModbusRequest writeSingleCoil(uint16_t address, bool on);
ModbusRequest writeSingleRegister(uint16_t address, uint16_t value);

// `packed` holds the coil states LSB-first and must remain valid until the
// request is encoded.
ModbusRequest writeMultipleCoils(uint16_t address, uint16_t quantity,
                                 const uint8_t* packed);

// `registersBE` holds `quantity` big-endian register pairs.
ModbusRequest writeMultipleRegisters(uint16_t address, uint16_t quantity,
                                     const uint8_t* registersBE);

ModbusRequest maskWriteRegister(uint16_t address, uint16_t andMask, uint16_t orMask);

ModbusRequest readWriteMultipleRegisters(uint16_t readAddress, uint16_t readQuantity,
                                         uint16_t writeAddress, uint16_t writeQuantity,
                                         const uint8_t* registersBE);

ModbusRequest readExceptionStatus();
ModbusRequest reportServerId();
ModbusRequest getCommEventCounter();
ModbusRequest getCommEventLog();

// Sub-function 0x0000 Return Query Data is the standard link test: whatever is
// sent comes back unchanged.
ModbusRequest diagnostics(uint16_t subFunction, const uint8_t* data, uint16_t dataLen);

ModbusRequest readFifoQueue(uint16_t pointerAddress);

// readDeviceIdCode: 01 basic, 02 regular, 03 extended, 04 one specific object.
ModbusRequest readDeviceIdentification(uint8_t readDeviceIdCode, uint8_t objectId);

} // namespace mbpdu
