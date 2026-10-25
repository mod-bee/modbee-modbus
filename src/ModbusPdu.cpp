#include "ModbusPdu.h"

#include <string.h>
#include "ModbusNoExcept.h"

const char* mb_decodeName(ModbusDecode result) {
    switch (result) {
        case ModbusDecode::Ok:            return "ok";
        case ModbusDecode::Short:         return "response too short";
        case ModbusDecode::Malformed:     return "response is self-inconsistent";
        case ModbusDecode::WrongFunction: return "response answers a different function";
        case ModbusDecode::Mismatch:      return "response echoes something the request did not send";
    }
    return "unknown";
}

uint16_t mb_expectedResponseBytes(const ModbusRequest& request) {
    switch (request.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
            // FC, byte count, the packed bits.
            return (uint16_t)(2 + mb_packedByteCount(request.quantity));

        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
        case MB_FC_READ_WRITE_REGISTERS:
            return (uint16_t)(2 + request.quantity * 2);

        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            return 5;                       // An echo of the address and value

        case MB_FC_MASK_WRITE_REGISTER:
            return 7;

        case MB_FC_READ_EXCEPTION_STATUS:
            return 2;

        case MB_FC_GET_COMM_EVENT_COUNTER:
            return 5;

        default:
            // Anything whose response cannot be sized from the request: a
            // device identification, a diagnostics echo, a report server id, a
            // vendor function code. The maximum, because a bound that is too
            // small here is a healthy device declared dead.
            return MB_MAX_PDU;
    }
}

uint16_t ModbusRequest::writeCount() const {
    switch (function) {
        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_MASK_WRITE_REGISTER:
            return 1;
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            return quantity;
        case MB_FC_READ_WRITE_REGISTERS:
            return writeQuantity;
        default:
            return 0;
    }
}

namespace mbpdu {

namespace {

// --- Writing helpers ---
//
// Every writer takes the remaining capacity and refuses rather than truncating.
// A PDU that was cut short is indistinguishable on the wire from one that was
// meant to be that length, so a partial write is never better than no write.

struct Writer {
    uint8_t* out;
    size_t capacity;
    size_t pos = 0;
    bool overflowed = false;

    Writer(uint8_t* buffer, size_t cap) : out(buffer), capacity(cap) {}

    void u8(uint8_t value) {
        if (pos + 1 > capacity) { overflowed = true; return; }
        out[pos++] = value;
    }

    void be16(uint16_t value) {
        if (pos + 2 > capacity) { overflowed = true; return; }
        mb_packBE16(&out[pos], value);
        pos += 2;
    }

    void bytes(const uint8_t* data, size_t length) {
        if (length == 0) return;
        if (!data || pos + length > capacity) { overflowed = true; return; }
        memcpy(&out[pos], data, length);
        pos += length;
    }

    // The finished length, or 0 if anything did not fit or the PDU grew past
    // what a PDU may be.
    size_t finish() const {
        if (overflowed || pos == 0 || pos > MB_MAX_PDU) return 0;
        return pos;
    }
};

// --- Reading helpers ---

// Bounds-checked cursor over the PDU being decoded. Every read goes through
// it, so there is one place that knows the length and one place to get it
// wrong. A peer's declared byte count is data, never a bound.
struct Reader {
    const uint8_t* in;
    size_t length;
    size_t pos = 0;

    Reader(const uint8_t* buffer, size_t len) : in(buffer), length(len) {}

    size_t remaining() const { return pos <= length ? length - pos : 0; }
    bool has(size_t count) const { return remaining() >= count; }

    uint8_t u8() { return pos < length ? in[pos++] : 0; }

    uint16_t be16() {
        if (pos + 2 > length) { pos = length; return 0; }
        const uint16_t value = mb_unpackBE16(&in[pos]);
        pos += 2;
        return value;
    }

    const uint8_t* take(size_t count) {
        if (pos + count > length) return nullptr;
        const uint8_t* start = &in[pos];
        pos += count;
        return start;
    }
};

} // namespace

// =============================================================================
// VALIDATION
// =============================================================================

uint8_t validateRequest(const ModbusRequest& req) {
    switch (req.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
            if (req.quantity < 1 || req.quantity > MB_MAX_READ_BITS) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (!mb_isValidAddressRange(req.address, req.quantity)) {
                return MB_EX_ILLEGAL_DATA_ADDRESS;
            }
            return MB_EX_NONE;

        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            if (req.quantity < 1 || req.quantity > MB_MAX_READ_REGISTERS) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (!mb_isValidAddressRange(req.address, req.quantity)) {
                return MB_EX_ILLEGAL_DATA_ADDRESS;
            }
            return MB_EX_NONE;

        case MB_FC_WRITE_SINGLE_COIL:
            // The specification allows two values and no others. A device that
            // quietly accepts 0x0001 as ON will read a neighbour's 0xFF00 as
            // OFF, so this is a real check.
            if (req.value != MB_COIL_ON && req.value != MB_COIL_OFF) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            return MB_EX_NONE;

        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_MASK_WRITE_REGISTER:
            // Any 16-bit value is legal, and a single address cannot run past
            // the end of the space.
            return MB_EX_NONE;

        case MB_FC_WRITE_MULTIPLE_COILS:
            if (req.quantity < 1 || req.quantity > MB_MAX_WRITE_BITS) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            // The payload must be exactly what the quantity implies. A byte
            // count that disagrees is the request lying about its own size,
            // which is how a parser that trusts it reads past the PDU.
            if (req.dataLen != mb_packedByteCount(req.quantity)) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (!mb_isValidAddressRange(req.address, req.quantity)) {
                return MB_EX_ILLEGAL_DATA_ADDRESS;
            }
            return MB_EX_NONE;

        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            if (req.quantity < 1 || req.quantity > MB_MAX_WRITE_REGISTERS) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (req.dataLen != (uint16_t)(req.quantity * 2)) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (!mb_isValidAddressRange(req.address, req.quantity)) {
                return MB_EX_ILLEGAL_DATA_ADDRESS;
            }
            return MB_EX_NONE;

        case MB_FC_READ_WRITE_REGISTERS:
            if (req.quantity < 1 || req.quantity > MB_MAX_RW_READ_REGISTERS) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            // 121, not 123: this request carries ten header bytes where 0x10
            // carries six, so four fewer bytes of payload fit.
            if (req.writeQuantity < 1 || req.writeQuantity > MB_MAX_RW_WRITE_REGISTERS) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (req.dataLen != (uint16_t)(req.writeQuantity * 2)) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            if (!mb_isValidAddressRange(req.address, req.quantity) ||
                !mb_isValidAddressRange(req.writeAddress, req.writeQuantity)) {
                return MB_EX_ILLEGAL_DATA_ADDRESS;
            }
            return MB_EX_NONE;

        case MB_FC_READ_EXCEPTION_STATUS:
        case MB_FC_GET_COMM_EVENT_COUNTER:
        case MB_FC_GET_COMM_EVENT_LOG:
        case MB_FC_REPORT_SERVER_ID:
            return MB_EX_NONE;   // No parameters to be wrong

        case MB_FC_DIAGNOSTICS:
            // Sub-function data is sub-function specific and mostly opaque
            // here; what is bounded is that it fits a PDU.
            if (req.dataLen > MB_MAX_PDU - 3) return MB_EX_ILLEGAL_DATA_VALUE;
            return MB_EX_NONE;

        case MB_FC_READ_FIFO_QUEUE:
            return MB_EX_NONE;

        case MB_FC_READ_FILE_RECORD:
        case MB_FC_WRITE_FILE_RECORD:
            // A byte count of 0x07..0xF5, and sub-requests that tile it
            // exactly. The sub-request walk lives in decodeRequest, which has
            // the bytes; here the envelope is what is checked.
            if (req.dataLen < 7 || req.dataLen > 0xF5) return MB_EX_ILLEGAL_DATA_VALUE;
            return MB_EX_NONE;

        case MB_FC_ENCAPSULATED_INTERFACE:
            if (req.meiType != MB_MEI_READ_DEVICE_ID) {
                // Including MB_MEI_CANOPEN, which is a real MEI type we do not
                // implement. Refusing it as a function is honest; answering it
                // as a device-identification request would not be.
                return MB_EX_ILLEGAL_FUNCTION;
            }
            if (req.readDeviceIdCode < 0x01 || req.readDeviceIdCode > 0x04) {
                return MB_EX_ILLEGAL_DATA_VALUE;
            }
            return MB_EX_NONE;

        default:
            return MB_EX_ILLEGAL_FUNCTION;
    }
}

// =============================================================================
// REQUEST ENCODING
// =============================================================================

static size_t encodeRequestInto(Writer& w, const ModbusRequest& req) {
    w.u8(req.function);

    switch (req.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            w.be16(req.address);
            w.be16(req.quantity);
            break;

        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
            w.be16(req.address);
            w.be16(req.value);
            break;

        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            w.be16(req.address);
            w.be16(req.quantity);
            // The byte count is written from the payload we actually hold, not
            // from a field the caller could set independently of it. There is
            // then no way for the two to disagree.
            w.u8((uint8_t)req.dataLen);
            w.bytes(req.data, req.dataLen);
            break;

        case MB_FC_MASK_WRITE_REGISTER:
            w.be16(req.address);
            w.be16(req.andMask);
            w.be16(req.orMask);
            break;

        case MB_FC_READ_WRITE_REGISTERS:
            w.be16(req.address);
            w.be16(req.quantity);
            w.be16(req.writeAddress);
            w.be16(req.writeQuantity);
            w.u8((uint8_t)req.dataLen);
            w.bytes(req.data, req.dataLen);
            break;

        case MB_FC_READ_EXCEPTION_STATUS:
        case MB_FC_GET_COMM_EVENT_COUNTER:
        case MB_FC_GET_COMM_EVENT_LOG:
        case MB_FC_REPORT_SERVER_ID:
            break;   // The function code is the whole request

        case MB_FC_DIAGNOSTICS:
            w.be16(req.subFunction);
            w.bytes(req.data, req.dataLen);
            break;

        case MB_FC_READ_FIFO_QUEUE:
            w.be16(req.address);
            break;

        case MB_FC_READ_FILE_RECORD:
        case MB_FC_WRITE_FILE_RECORD:
            w.u8((uint8_t)req.dataLen);
            w.bytes(req.data, req.dataLen);
            break;

        case MB_FC_ENCAPSULATED_INTERFACE:
            w.u8(req.meiType);
            if (req.meiType == MB_MEI_READ_DEVICE_ID) {
                w.u8(req.readDeviceIdCode);
                w.u8(req.objectId);
            } else {
                w.bytes(req.data, req.dataLen);
            }
            break;

        default:
            return 0;
    }

    return w.finish();
}

size_t encodeRequest(uint8_t* out, size_t capacity, const ModbusRequest& req) {
    if (!out) return 0;
    if (validateRequest(req) != MB_EX_NONE) return 0;
    Writer w(out, capacity);
    return encodeRequestInto(w, req);
}

size_t requestSize(const ModbusRequest& req) {
    // Sized by encoding into a scratch PDU rather than by a second table of
    // lengths that could drift from the encoder. One source of truth.
    uint8_t scratch[MB_MAX_PDU];
    return encodeRequest(scratch, sizeof scratch, req);
}

// =============================================================================
// RESPONSE ENCODING
// =============================================================================

size_t encodeReadResponse(uint8_t* out, size_t capacity, uint8_t function,
                          const uint8_t* payload, size_t payloadLen) {
    if (!out || !payload) return 0;
    if (payloadLen == 0 || payloadLen > MB_MAX_BYTE_COUNT) return 0;

    Writer w(out, capacity);
    w.u8(function);
    w.u8((uint8_t)payloadLen);
    w.bytes(payload, payloadLen);
    return w.finish();
}

size_t encodeEchoResponse(uint8_t* out, size_t capacity, const ModbusRequest& req) {
    if (!out) return 0;

    Writer w(out, capacity);
    w.u8(req.function);

    switch (req.function) {
        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
            w.be16(req.address);
            w.be16(req.value);
            break;

        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            w.be16(req.address);
            w.be16(req.quantity);
            break;

        case MB_FC_MASK_WRITE_REGISTER:
            w.be16(req.address);
            w.be16(req.andMask);
            w.be16(req.orMask);
            break;

        default:
            return 0;   // Not a function whose response is an echo
    }

    return w.finish();
}

size_t encodeRawResponse(uint8_t* out, size_t capacity, uint8_t function,
                         const uint8_t* body, size_t bodyLen) {
    if (!out) return 0;
    Writer w(out, capacity);
    w.u8(function);
    w.bytes(body, bodyLen);
    return w.finish();
}

size_t encodeException(uint8_t* out, size_t capacity, uint8_t function,
                       uint8_t exceptionCode) {
    if (!out || capacity < 2) return 0;
    // MB_EX_NONE is not an exception and MB_EX_DECODE_FAILED is not a wire
    // value; encoding either would put a byte on the line that means nothing.
    if (exceptionCode == MB_EX_NONE || exceptionCode == MB_EX_DECODE_FAILED) return 0;

    out[0] = mb_exceptionFunction(function);
    out[1] = exceptionCode;
    return 2;
}

// =============================================================================
// REQUEST DECODING
// =============================================================================

uint8_t decodeRequest(const uint8_t* pdu, size_t length, ModbusRequest& out) {
    if (!pdu || length < 1) return MB_EX_DECODE_FAILED;

    out = ModbusRequest();
    Reader r(pdu, length);
    out.function = r.u8();

    // A request never carries the exception bit. One that does is a response
    // that has arrived where a request was expected - on a shared serial bus,
    // usually another master's traffic - and answering it would put a second
    // response on the line.
    if (mb_isExceptionFunction(out.function)) return MB_EX_DECODE_FAILED;

    switch (out.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS:
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            if (length != 5) return MB_EX_DECODE_FAILED;
            out.address = r.be16();
            out.quantity = r.be16();
            break;

        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
            if (length != 5) return MB_EX_DECODE_FAILED;
            out.address = r.be16();
            out.value = r.be16();
            out.quantity = 1;
            break;

        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS: {
            // Six, not seven: a quantity of zero with a byte count of zero is
            // a whole request, and the answer to it is an illegal data value
            // rather than silence.
            if (length < 6) return MB_EX_DECODE_FAILED;
            out.address = r.be16();
            out.quantity = r.be16();
            const uint8_t byteCount = r.u8();

            // The declared byte count must be exactly the rest of the PDU. It
            // is checked against the bytes actually present BEFORE it is used
            // to take any, because a byte count is data the sender chose and a
            // parser that treats it as a bound reads whatever follows the PDU
            // in memory.
            if (byteCount != r.remaining()) return MB_EX_DECODE_FAILED;
            out.data = r.take(byteCount);
            out.dataLen = byteCount;
            break;
        }

        case MB_FC_MASK_WRITE_REGISTER:
            if (length != 7) return MB_EX_DECODE_FAILED;
            out.address = r.be16();
            out.andMask = r.be16();
            out.orMask = r.be16();
            out.quantity = 1;
            break;

        case MB_FC_READ_WRITE_REGISTERS: {
            if (length < 10) return MB_EX_DECODE_FAILED;   // As above, for a write of zero
            out.address = r.be16();
            out.quantity = r.be16();
            out.writeAddress = r.be16();
            out.writeQuantity = r.be16();
            const uint8_t byteCount = r.u8();
            if (byteCount != r.remaining()) return MB_EX_DECODE_FAILED;
            out.data = r.take(byteCount);
            out.dataLen = byteCount;
            break;
        }

        case MB_FC_READ_EXCEPTION_STATUS:
        case MB_FC_GET_COMM_EVENT_COUNTER:
        case MB_FC_GET_COMM_EVENT_LOG:
        case MB_FC_REPORT_SERVER_ID:
            if (length != 1) return MB_EX_DECODE_FAILED;
            break;

        case MB_FC_DIAGNOSTICS:
            if (length < 3) return MB_EX_DECODE_FAILED;
            out.subFunction = r.be16();
            out.dataLen = (uint16_t)r.remaining();
            out.data = out.dataLen ? r.take(out.dataLen) : nullptr;
            break;

        case MB_FC_READ_FIFO_QUEUE:
            if (length != 3) return MB_EX_DECODE_FAILED;
            out.address = r.be16();
            break;

        case MB_FC_READ_FILE_RECORD:
        case MB_FC_WRITE_FILE_RECORD: {
            if (length < 2) return MB_EX_DECODE_FAILED;
            const uint8_t byteCount = r.u8();
            if (byteCount != r.remaining()) return MB_EX_DECODE_FAILED;
            out.data = r.take(byteCount);
            out.dataLen = byteCount;
            break;
        }

        case MB_FC_ENCAPSULATED_INTERFACE:
            if (length < 2) return MB_EX_DECODE_FAILED;
            out.meiType = r.u8();
            if (out.meiType == MB_MEI_READ_DEVICE_ID) {
                if (length != 4) return MB_EX_DECODE_FAILED;
                out.readDeviceIdCode = r.u8();
                out.objectId = r.u8();
            } else {
                out.dataLen = (uint16_t)r.remaining();
                out.data = out.dataLen ? r.take(out.dataLen) : nullptr;
            }
            break;

        default:
            // Intelligible enough to answer: we know which function was asked
            // for and that we do not have it. Reported as an illegal function
            // and not as an illegal data value.
            //
            // The body is still handed back. A server may have an application
            // handler for this code - a vendor function, or one of the three
            // public codes whose storage is device-specific - and that handler
            // needs the bytes. A caller with no handler ignores them and
            // refuses, exactly as before.
            out.dataLen = (uint16_t)(length - 1);
            out.data = out.dataLen ? pdu + 1 : nullptr;
            return MB_EX_ILLEGAL_FUNCTION;
    }

    return validateRequest(out);
}

// =============================================================================
// RESPONSE DECODING
// =============================================================================

ModbusDecode decodeResponse(const uint8_t* pdu, size_t length,
                            const ModbusRequest& req, ModbusResponse& out) {
    out = ModbusResponse();
    if (!pdu || length < 1) return ModbusDecode::Short;

    Reader r(pdu, length);
    out.function = r.u8();

    // An exception first, always. Everything below reads fields an exception
    // response does not have, and noticing the exception one step too late
    // retires the read as though it had succeeded.
    if (mb_isExceptionFunction(out.function)) {
        if (length != 2) return ModbusDecode::Malformed;
        if (mb_baseFunction(out.function) != req.function) {
            return ModbusDecode::WrongFunction;
        }
        out.isException = true;
        out.exceptionCode = r.u8();
        return ModbusDecode::Ok;
    }

    if (out.function != req.function) return ModbusDecode::WrongFunction;

    switch (req.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS: {
            if (length < 2) return ModbusDecode::Short;
            const uint8_t byteCount = r.u8();
            if (byteCount != r.remaining()) return ModbusDecode::Malformed;

            // The wire carries bytes; the client asked for bits. Only the
            // request knows how many, which is why it is a parameter here.
            // Deriving the count as byteCount * 8 would over-report by up to
            // seven every time.
            if (byteCount != mb_packedByteCount(req.quantity)) {
                return ModbusDecode::Mismatch;
            }
            out.data = r.take(byteCount);
            out.dataLen = byteCount;
            out.quantity = req.quantity;
            break;
        }

        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
        case MB_FC_READ_WRITE_REGISTERS: {
            if (length < 2) return ModbusDecode::Short;
            const uint8_t byteCount = r.u8();
            if (byteCount != r.remaining()) return ModbusDecode::Malformed;
            if (byteCount % 2 != 0) return ModbusDecode::Malformed;
            if (byteCount != (uint8_t)(req.quantity * 2)) return ModbusDecode::Mismatch;
            out.data = r.take(byteCount);
            out.dataLen = byteCount;
            out.quantity = req.quantity;
            break;
        }

        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
            if (length != 5) return ModbusDecode::Short;
            out.address = r.be16();
            out.value = r.be16();
            // A server that echoes something else has done something other
            // than what was asked. Reporting success here would tell the
            // application a setpoint landed that did not.
            if (out.address != req.address || out.value != req.value) {
                return ModbusDecode::Mismatch;
            }
            out.quantity = 1;
            break;

        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            if (length != 5) return ModbusDecode::Short;
            out.address = r.be16();
            out.value = r.be16();          // Quantity written
            if (out.address != req.address || out.value != req.quantity) {
                return ModbusDecode::Mismatch;
            }
            out.quantity = req.quantity;
            break;

        case MB_FC_MASK_WRITE_REGISTER:
            if (length != 7) return ModbusDecode::Short;
            out.address = r.be16();
            out.andMask = r.be16();
            out.orMask = r.be16();
            if (out.address != req.address || out.andMask != req.andMask ||
                out.orMask != req.orMask) {
                return ModbusDecode::Mismatch;
            }
            out.quantity = 1;
            break;

        case MB_FC_READ_EXCEPTION_STATUS:
            // [FC][status]
            if (length != 2) return ModbusDecode::Short;
            out.data = r.take(1);
            out.dataLen = 1;
            break;

        case MB_FC_DIAGNOSTICS: {
            // [FC][subFunction][data...]. Echoed back for Return Query Data,
            // which is the point of that sub-function.
            if (length < 3) return ModbusDecode::Short;
            const uint16_t echoed = r.be16();
            if (echoed != req.subFunction) return ModbusDecode::Mismatch;
            out.dataLen = (uint16_t)r.remaining();
            out.data = out.dataLen ? r.take(out.dataLen) : nullptr;
            break;
        }

        case MB_FC_GET_COMM_EVENT_COUNTER:
            // [FC][status:2][eventCount:2]
            if (length != 5) return ModbusDecode::Short;
            out.data = r.take(4);
            out.dataLen = 4;
            break;

        case MB_FC_GET_COMM_EVENT_LOG:
        case MB_FC_REPORT_SERVER_ID:
        case MB_FC_READ_FILE_RECORD:
        case MB_FC_WRITE_FILE_RECORD: {
            // [FC][byteCount][body]
            if (length < 2) return ModbusDecode::Short;
            const uint8_t byteCount = r.u8();
            if (byteCount != r.remaining()) return ModbusDecode::Malformed;
            out.data = r.take(byteCount);
            out.dataLen = byteCount;
            break;
        }

        case MB_FC_READ_FIFO_QUEUE: {
            // [FC][byteCount:2][fifoCount:2][values...]. The only read
            // response with a two-byte count, and the two counts must agree:
            // byteCount covers fifoCount plus the values.
            if (length < 5) return ModbusDecode::Short;
            const uint16_t byteCount = r.be16();
            const uint16_t fifoCount = r.be16();
            if (byteCount != (uint16_t)(2 + fifoCount * 2)) return ModbusDecode::Malformed;
            if ((size_t)(fifoCount * 2) != r.remaining()) return ModbusDecode::Malformed;
            if (fifoCount > 31) return ModbusDecode::Malformed;   // Spec maximum
            out.data = fifoCount ? r.take((size_t)fifoCount * 2) : nullptr;
            out.dataLen = (uint16_t)(fifoCount * 2);
            out.quantity = fifoCount;
            break;
        }

        case MB_FC_ENCAPSULATED_INTERFACE: {
            // [FC][MEI type][...]. The body is walked by the caller, which is
            // the only party that knows what it asked for; what is checked
            // here is that the MEI type matches, because a device answering a
            // different encapsulated protocol is not answering us.
            if (length < 2) return ModbusDecode::Short;
            const uint8_t meiType = r.u8();
            if (meiType != req.meiType) return ModbusDecode::Mismatch;
            out.dataLen = (uint16_t)r.remaining();
            out.data = out.dataLen ? r.take(out.dataLen) : nullptr;
            break;
        }

        default:
            return ModbusDecode::WrongFunction;
    }

    return ModbusDecode::Ok;
}

// =============================================================================
// CONVENIENCE BUILDERS
// =============================================================================

namespace {
ModbusRequest readOf(uint8_t function, uint16_t address, uint16_t quantity) {
    ModbusRequest req;
    req.function = function;
    req.address = address;
    req.quantity = quantity;
    return req;
}
} // namespace

ModbusRequest readCoils(uint16_t a, uint16_t q)            { return readOf(MB_FC_READ_COILS, a, q); }
ModbusRequest readDiscreteInputs(uint16_t a, uint16_t q)   { return readOf(MB_FC_READ_DISCRETE_INPUTS, a, q); }
ModbusRequest readHoldingRegisters(uint16_t a, uint16_t q) { return readOf(MB_FC_READ_HOLDING_REGISTERS, a, q); }
ModbusRequest readInputRegisters(uint16_t a, uint16_t q)   { return readOf(MB_FC_READ_INPUT_REGISTERS, a, q); }

ModbusRequest writeSingleCoil(uint16_t address, bool on) {
    ModbusRequest req;
    req.function = MB_FC_WRITE_SINGLE_COIL;
    req.address = address;
    req.value = on ? MB_COIL_ON : MB_COIL_OFF;
    req.quantity = 1;
    return req;
}

ModbusRequest writeSingleRegister(uint16_t address, uint16_t value) {
    ModbusRequest req;
    req.function = MB_FC_WRITE_SINGLE_REGISTER;
    req.address = address;
    req.value = value;
    req.quantity = 1;
    return req;
}

ModbusRequest writeMultipleCoils(uint16_t address, uint16_t quantity,
                                 const uint8_t* packed) {
    ModbusRequest req;
    req.function = MB_FC_WRITE_MULTIPLE_COILS;
    req.address = address;
    req.quantity = quantity;
    req.data = packed;
    req.dataLen = mb_packedByteCount(quantity);
    return req;
}

ModbusRequest writeMultipleRegisters(uint16_t address, uint16_t quantity,
                                     const uint8_t* registersBE) {
    ModbusRequest req;
    req.function = MB_FC_WRITE_MULTIPLE_REGISTERS;
    req.address = address;
    req.quantity = quantity;
    req.data = registersBE;
    req.dataLen = (uint16_t)(quantity * 2);
    return req;
}

ModbusRequest maskWriteRegister(uint16_t address, uint16_t andMask, uint16_t orMask) {
    ModbusRequest req;
    req.function = MB_FC_MASK_WRITE_REGISTER;
    req.address = address;
    req.andMask = andMask;
    req.orMask = orMask;
    req.quantity = 1;
    return req;
}

ModbusRequest readWriteMultipleRegisters(uint16_t readAddress, uint16_t readQuantity,
                                         uint16_t writeAddress, uint16_t writeQuantity,
                                         const uint8_t* registersBE) {
    ModbusRequest req;
    req.function = MB_FC_READ_WRITE_REGISTERS;
    req.address = readAddress;
    req.quantity = readQuantity;
    req.writeAddress = writeAddress;
    req.writeQuantity = writeQuantity;
    req.data = registersBE;
    req.dataLen = (uint16_t)(writeQuantity * 2);
    return req;
}

namespace {
ModbusRequest bare(uint8_t function) {
    ModbusRequest req;
    req.function = function;
    return req;
}
} // namespace

ModbusRequest readExceptionStatus()  { return bare(MB_FC_READ_EXCEPTION_STATUS); }
ModbusRequest reportServerId()       { return bare(MB_FC_REPORT_SERVER_ID); }
ModbusRequest getCommEventCounter()  { return bare(MB_FC_GET_COMM_EVENT_COUNTER); }
ModbusRequest getCommEventLog()      { return bare(MB_FC_GET_COMM_EVENT_LOG); }

ModbusRequest diagnostics(uint16_t subFunction, const uint8_t* data, uint16_t dataLen) {
    ModbusRequest req;
    req.function = MB_FC_DIAGNOSTICS;
    req.subFunction = subFunction;
    req.data = data;
    req.dataLen = dataLen;
    return req;
}

ModbusRequest readFifoQueue(uint16_t pointerAddress) {
    ModbusRequest req;
    req.function = MB_FC_READ_FIFO_QUEUE;
    req.address = pointerAddress;
    return req;
}

ModbusRequest readDeviceIdentification(uint8_t readDeviceIdCode, uint8_t objectId) {
    ModbusRequest req;
    req.function = MB_FC_ENCAPSULATED_INTERFACE;
    req.meiType = MB_MEI_READ_DEVICE_ID;
    req.readDeviceIdCode = readDeviceIdCode;
    req.objectId = objectId;
    return req;
}

} // namespace mbpdu
