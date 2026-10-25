#include "ModbusServerEngine.h"

#include <string.h>
#include "ModbusNoExcept.h"

// Diagnostics sub-functions. Only the ones a device is actually asked for.
#define MB_DIAG_RETURN_QUERY_DATA        0x0000
#define MB_DIAG_RESTART_COMMUNICATIONS   0x0001
#define MB_DIAG_RETURN_DIAGNOSTIC_REG    0x0002
#define MB_DIAG_FORCE_LISTEN_ONLY        0x0004
#define MB_DIAG_CLEAR_COUNTERS           0x000A
#define MB_DIAG_BUS_MESSAGE_COUNT        0x000B
#define MB_DIAG_BUS_COMM_ERROR_COUNT     0x000C
#define MB_DIAG_BUS_EXCEPTION_COUNT      0x000D
#define MB_DIAG_SERVER_MESSAGE_COUNT     0x000E
#define MB_DIAG_SERVER_NO_RESPONSE_COUNT 0x000F
#define MB_DIAG_SERVER_NAK_COUNT         0x0010
#define MB_DIAG_SERVER_BUSY_COUNT        0x0011
#define MB_DIAG_CHARACTER_OVERRUN_COUNT  0x0012

void ModbusServerEngine::setFunctionHandler(uint8_t function, ModbusFunctionHandler handler) {
    for (size_t i = 0; i < _handlers.size(); i++) {
        if (_handlers[i].function != function) continue;
        if (handler) _handlers[i].handler = handler;
        else _handlers.erase(i);
        return;
    }
    if (!handler) return;
    FunctionEntry entry;
    entry.function = function;
    entry.handler = handler;
    _handlers.push(entry);
}

bool ModbusServerEngine::hasFunctionHandler(uint8_t function) const {
    return findHandler(function) != nullptr;
}

const ModbusFunctionHandler* ModbusServerEngine::findHandler(uint8_t function) const {
    for (size_t i = 0; i < _handlers.size(); i++) {
        if (_handlers[i].function == function) return &_handlers[i].handler;
    }
    return nullptr;
}

size_t ModbusServerEngine::refuse(uint8_t function, uint8_t exceptionCode,
                                  uint8_t* out, size_t capacity) {
    _counters.busExceptionError++;
    return mbpdu::encodeException(out, capacity, function, exceptionCode);
}

// =============================================================================
// THE ENTRY POINT
// =============================================================================

size_t ModbusServerEngine::handle(uint8_t unit, bool broadcast,
                                  const uint8_t* pdu, size_t pduLen,
                                  uint8_t* out, size_t capacity) {
    _counters.busMessage++;
    _counters.serverMessage++;

    ModbusRequest req;
    const uint8_t decoded = mbpdu::decodeRequest(pdu, pduLen, req);

    // Unintelligible. On a serial line the correct answer is SILENCE: we do not
    // know what was asked, so we cannot say what was wrong with it, and
    // guessing a function code to attach an exception to would answer a request
    // that may have been addressed to somebody else.
    if (decoded == MB_EX_DECODE_FAILED) {
        _counters.serverNoResponse++;
        return 0;
    }

    // Listen-only mode (entered by diagnostics sub-function 0x04) receives and
    // acts on nothing and answers nothing. Only a Restart Communications
    // (0x08/0x0001) leaves it, and that one is processed even here - otherwise
    // the mode could never be left.
    if (_counters.listenOnly) {
        const bool isRestart = req.function == MB_FC_DIAGNOSTICS &&
                               req.subFunction == MB_DIAG_RESTART_COMMUNICATIONS;
        if (!isRestart) {
            _counters.serverNoResponse++;
            return 0;
        }
    }

    // Serial-only function codes have no meaning on TCP: they describe a shared
    // bus and a single device behind one address, and a TCP server is neither.
    if (!_serialLine && mb_isSerialOnlyFunction(req.function)) {
        if (broadcast) { _counters.serverNoResponse++; return 0; }
        return refuse(req.function, MB_EX_ILLEGAL_FUNCTION, out, capacity);
    }

    // A broadcast may only WRITE. There is no response to a broadcast, so a
    // broadcast read asks a question nobody is permitted to answer.
    if (broadcast && !mb_isWriteFunction(req.function)) {
        _counters.serverNoResponse++;
        return 0;
    }

    // A function code we do not implement is not necessarily one the SERVER
    // does not implement. Checked before the refusal, so an application handler
    // gets its chance at a vendor code, or at one of the three public codes
    // whose storage is device-specific.
    const ModbusFunctionHandler* handler = findHandler(req.function);

    if (decoded != MB_EX_NONE && !(decoded == MB_EX_ILLEGAL_FUNCTION && handler)) {
        if (broadcast) { _counters.serverNoResponse++; return 0; }
        return refuse(req.function, decoded, out, capacity);
    }

    // The application's own rules, before anything is applied. This is where a
    // panel in local mode refuses writes, and the only place that can be said
    // before the write has already happened.
    if (_filter) {
        const uint8_t refusal = _filter(unit, req);
        if (refusal != MB_EX_NONE) {
            if (broadcast) { _counters.serverNoResponse++; return 0; }
            if (refusal == MB_EX_SERVER_DEVICE_BUSY) _counters.serverBusy++;
            if (refusal == MB_EX_NEGATIVE_ACKNOWLEDGE) _counters.serverNak++;
            return refuse(req.function, refusal, out, capacity);
        }
    }

    size_t responseLen = 0;
    uint8_t exception = MB_EX_NONE;

    // The application's own implementation. It overrides a built-in one where
    // both exist, which is how a device with a real file store or an unusual
    // diagnostics set provides its own.
    if (handler) {
        uint8_t body[MB_MAX_PDU];
        uint8_t refusal = MB_EX_NONE;
        const size_t bodyLen = (*handler)(unit, req, body, sizeof body - 1, refusal);

        if (refusal != MB_EX_NONE) {
            if (broadcast) { _counters.serverNoResponse++; return 0; }
            return refuse(req.function, refusal, out, capacity);
        }
        if (bodyLen == 0 || broadcast) {
            // Silence, by the handler's choice or because a broadcast is never
            // answered.
            _counters.serverNoResponse++;
            return 0;
        }
        _counters.eventCount++;
        return mbpdu::encodeRawResponse(out, capacity, req.function, body, bodyLen);
    }

    switch (req.function) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_DISCRETE_INPUTS: {
            uint8_t byteCount = 0;
            exception = readBits(req, mb_tableForFunction(req.function), byteCount);
            if (exception == MB_EX_NONE) {
                responseLen = mbpdu::encodeReadResponse(out, capacity, req.function,
                                                        _scratch, byteCount);
            }
            break;
        }

        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS: {
            uint8_t byteCount = 0;
            exception = readRegisters(req, mb_tableForFunction(req.function), byteCount);
            if (exception == MB_EX_NONE) {
                responseLen = mbpdu::encodeReadResponse(out, capacity, req.function,
                                                        _scratch, byteCount);
            }
            break;
        }

        case MB_FC_WRITE_SINGLE_COIL:
        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_WRITE_MULTIPLE_COILS:
        case MB_FC_WRITE_MULTIPLE_REGISTERS:
        case MB_FC_MASK_WRITE_REGISTER:
            exception = applyWrite(unit, req);
            if (exception == MB_EX_NONE && !broadcast) {
                responseLen = mbpdu::encodeEchoResponse(out, capacity, req);
            }
            break;

        case MB_FC_READ_WRITE_REGISTERS: {
            // The specification is explicit: the WRITE is performed before the
            // read. A client using 0x17 to set a command and read back the
            // result depends on that order, and doing it the other way round
            // returns the value from before the command it just sent.
            //
            // The read range is checked BEFORE the write, though, because an
            // exception after the write would tell the client nothing was done
            // when its registers had in fact been changed.
            if (!_map.hasRange(ModbusTable::HoldingRegister, req.address, req.quantity)) {
                exception = MB_EX_ILLEGAL_DATA_ADDRESS;
                break;
            }
            exception = applyWrite(unit, req);
            if (exception != MB_EX_NONE) break;

            uint8_t byteCount = 0;
            exception = readRegisters(req, ModbusTable::HoldingRegister, byteCount);
            if (exception == MB_EX_NONE) {
                responseLen = mbpdu::encodeReadResponse(out, capacity, req.function,
                                                        _scratch, byteCount);
            }
            break;
        }

        case MB_FC_READ_EXCEPTION_STATUS: {
            // Eight coils' worth of device-specific status. Taken from coils
            // 0..7 where they are bound, which is the convention, and zero
            // where they are not.
            uint8_t status = 0;
            for (uint16_t i = 0; i < 8; i++) {
                bool bit = false;
                if (_map.readBit(ModbusTable::Coil, i, bit) && bit) {
                    status |= (uint8_t)(1u << i);
                }
            }
            responseLen = mbpdu::encodeRawResponse(out, capacity, req.function, &status, 1);
            break;
        }

        case MB_FC_DIAGNOSTICS: {
            bool silent = false;
            responseLen = diagnostics(req, out, capacity, silent);
            if (silent) return 0;   // Answered with nothing, by design
            if (responseLen == 0 && !broadcast) exception = MB_EX_ILLEGAL_DATA_VALUE;
            break;
        }

        case MB_FC_GET_COMM_EVENT_COUNTER: {
            // [status:2][eventCount:2]. Status is 0xFFFF while a long-duration
            // program command is running and 0x0000 otherwise.
            uint8_t body[4];
            mb_packBE16(body, 0x0000);
            mb_packBE16(body + 2, _counters.eventCount);
            responseLen = mbpdu::encodeRawResponse(out, capacity, req.function, body, 4);
            break;
        }

        case MB_FC_GET_COMM_EVENT_LOG: {
            // [byteCount][status:2][eventCount:2][messageCount:2][events...].
            // The event log itself is optional and we keep none, so the byte
            // count covers the three counters and nothing else.
            uint8_t body[7];
            body[0] = 6;
            mb_packBE16(body + 1, 0x0000);
            mb_packBE16(body + 3, _counters.eventCount);
            mb_packBE16(body + 5, _counters.busMessage);
            responseLen = mbpdu::encodeRawResponse(out, capacity, req.function, body, 7);
            break;
        }

        case MB_FC_REPORT_SERVER_ID:
            responseLen = reportServerId(out, capacity);
            break;

        case MB_FC_ENCAPSULATED_INTERFACE: {
            uint8_t refusal = MB_EX_ILLEGAL_DATA_VALUE;
            responseLen = deviceIdentification(req, out, capacity, refusal);
            if (responseLen == 0) exception = refusal;
            break;
        }

        default:
            exception = MB_EX_ILLEGAL_FUNCTION;
            break;
    }

    if (broadcast) {
        _counters.serverNoResponse++;
        return 0;
    }

    if (exception != MB_EX_NONE) {
        return refuse(req.function, exception, out, capacity);
    }

    if (responseLen == 0) {
        // Everything above either produced a response or set an exception. A
        // zero here means the response would not fit the buffer it was given,
        // which is a device failure rather than a fault in the request.
        return refuse(req.function, MB_EX_SERVER_DEVICE_FAILURE, out, capacity);
    }

    _counters.eventCount++;
    return responseLen;
}

// =============================================================================
// READS
// =============================================================================

uint8_t ModbusServerEngine::readBits(const ModbusRequest& req, ModbusTable table,
                                     uint8_t& byteCount) {
    // The WHOLE range, checked before a single bit is gathered. A read of ten
    // coils where three are unbound is one illegal data address, not a partial
    // response - there is no way to say "these three are missing" in a Modbus
    // reply, so answering with zeroes would be inventing data.
    if (!_map.hasRange(table, req.address, req.quantity)) {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }

    byteCount = (uint8_t)mb_packedByteCount(req.quantity);
    memset(_scratch, 0, byteCount);

    for (uint16_t i = 0; i < req.quantity; i++) {
        bool value = false;
        if (!_map.readBit(table, (uint16_t)(req.address + i), value)) {
            return MB_EX_SERVER_DEVICE_FAILURE;
        }
        if (value) mb_setPackedBit(_scratch, i, true);
    }
    return MB_EX_NONE;
}

uint8_t ModbusServerEngine::readRegisters(const ModbusRequest& req, ModbusTable table,
                                          uint8_t& byteCount) {
    if (!_map.hasRange(table, req.address, req.quantity)) {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }

    // A byte count is one byte, so no read response may declare more than 255.
    // validateRequest already caps the quantity at 125, but the bound belongs
    // here too rather than being inherited from a caller.
    if ((uint32_t)req.quantity * 2 > MB_MAX_BYTE_COUNT) return MB_EX_ILLEGAL_DATA_VALUE;
    byteCount = (uint8_t)(req.quantity * 2);

    for (uint16_t i = 0; i < req.quantity; i++) {
        uint16_t value = 0;
        if (!_map.readRegister(table, (uint16_t)(req.address + i), value)) {
            return MB_EX_SERVER_DEVICE_FAILURE;
        }
        mb_packBE16(_scratch + (size_t)i * 2, value);
    }
    return MB_EX_NONE;
}

// =============================================================================
// WRITES
// =============================================================================

uint8_t ModbusServerEngine::applyWrite(uint8_t unit, const ModbusRequest& req) {
    const uint16_t target = req.writeTarget();
    const uint16_t count = req.writeCount();
    const ModbusTable table = req.function == MB_FC_READ_WRITE_REGISTERS
                                  ? ModbusTable::HoldingRegister
                                  : mb_tableForFunction(req.function);

    // Validated in full BEFORE any of it is applied.
    //
    // A block write that fails partway leaves the device in a state the client
    // never asked for and cannot discover - it is told the whole thing failed,
    // while half of it happened.
    if (!_map.hasRange(table, target, count)) {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    if (!mb_tableIsWritable(table)) {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    // Bound is not enough: a read-only handler in the middle of the range
    // would refuse its part after the bindings before it had been written.
    if (!_map.hasWritableRange(table, target, count)) {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }

    switch (req.function) {
        case MB_FC_WRITE_SINGLE_COIL:
            if (!_map.writeBit(table, req.address, req.value == MB_COIL_ON, unit)) {
                return MB_EX_ILLEGAL_DATA_ADDRESS;
            }
            break;

        case MB_FC_WRITE_SINGLE_REGISTER:
            if (!_map.writeRegister(table, req.address, req.value, unit)) {
                return MB_EX_ILLEGAL_DATA_ADDRESS;
            }
            break;

        case MB_FC_WRITE_MULTIPLE_COILS:
            for (uint16_t i = 0; i < req.quantity; i++) {
                const bool value = mb_getPackedBit(req.data, i);
                if (!_map.writeBit(table, (uint16_t)(req.address + i), value, unit)) {
                    return MB_EX_ILLEGAL_DATA_ADDRESS;
                }
            }
            break;

        case MB_FC_WRITE_MULTIPLE_REGISTERS:
            for (uint16_t i = 0; i < req.quantity; i++) {
                const uint16_t value = mb_unpackBE16(req.data + (size_t)i * 2);
                if (!_map.writeRegister(table, (uint16_t)(req.address + i), value, unit)) {
                    return MB_EX_ILLEGAL_DATA_ADDRESS;
                }
            }
            break;

        case MB_FC_MASK_WRITE_REGISTER: {
            // Result = (Current AND And_Mask) OR (Or_Mask AND NOT And_Mask)
            //
            // The only atomic read-modify-write Modbus has. Without it, two
            // clients setting different bits of one register corrupt each
            // other: each reads, modifies and writes back, and whichever writes
            // second erases the first's bit.
            uint16_t current = 0;
            if (!_map.readRegister(table, req.address, current)) {
                return MB_EX_ILLEGAL_DATA_ADDRESS;
            }
            const uint16_t result =
                (uint16_t)((current & req.andMask) | (req.orMask & (uint16_t)~req.andMask));
            if (!_map.writeRegister(table, req.address, result, unit)) {
                return MB_EX_ILLEGAL_DATA_ADDRESS;
            }
            break;
        }

        case MB_FC_READ_WRITE_REGISTERS:
            for (uint16_t i = 0; i < req.writeQuantity; i++) {
                const uint16_t value = mb_unpackBE16(req.data + (size_t)i * 2);
                if (!_map.writeRegister(table, (uint16_t)(req.writeAddress + i), value, unit)) {
                    return MB_EX_ILLEGAL_DATA_ADDRESS;
                }
            }
            break;

        default:
            return MB_EX_ILLEGAL_FUNCTION;
    }

    // Everything the request changed is reported now that ALL of it has been
    // applied, so a handler never sees a half-written block.
    _map.dispatchChangeEvents();
    return MB_EX_NONE;
}

// =============================================================================
// DIAGNOSTICS
// =============================================================================

size_t ModbusServerEngine::diagnostics(const ModbusRequest& req, uint8_t* out,
                                       size_t capacity, bool& silent) {
    silent = false;
    uint8_t body[MB_MAX_PDU];
    size_t bodyLen = 0;

    // Every sub-function echoes its own number first.
    mb_packBE16(body, req.subFunction);
    bodyLen = 2;

    switch (req.subFunction) {
        case MB_DIAG_RETURN_QUERY_DATA:
            // Whatever was sent comes back unchanged. The standard link test,
            // and the only sub-function every device is expected to answer.
            if (req.dataLen > sizeof body - 2) return 0;
            memcpy(body + 2, req.data, req.dataLen);
            bodyLen += req.dataLen;
            break;

        case MB_DIAG_RESTART_COMMUNICATIONS:
            // Leaves listen-only mode, and optionally clears the counters. The
            // request data is 0xFF00 to clear the log, 0x0000 to keep it.
            _counters.listenOnly = false;
            if (req.dataLen >= 2 && mb_unpackBE16(req.data) == 0xFF00) {
                _counters = ModbusServerCounters();
            }
            if (req.dataLen > sizeof body - 2) return 0;
            memcpy(body + 2, req.data, req.dataLen);
            bodyLen += req.dataLen;
            break;

        case MB_DIAG_FORCE_LISTEN_ONLY:
            // Receives everything, acts on nothing, answers nothing - INCLUDING
            // this request. The specification is explicit that no response is
            // returned, which is why this is SILENT rather than an echo, and
            // why silence has to be distinguishable from a failure to build one.
            _counters.listenOnly = true;
            silent = true;
            return 0;

        case MB_DIAG_CLEAR_COUNTERS: {
            const bool listenOnly = _counters.listenOnly;
            _counters = ModbusServerCounters();
            _counters.listenOnly = listenOnly;
            mb_packBE16(body + 2, 0x0000);
            bodyLen += 2;
            break;
        }

        case MB_DIAG_RETURN_DIAGNOSTIC_REG:
        case MB_DIAG_BUS_MESSAGE_COUNT:
        case MB_DIAG_BUS_COMM_ERROR_COUNT:
        case MB_DIAG_BUS_EXCEPTION_COUNT:
        case MB_DIAG_SERVER_MESSAGE_COUNT:
        case MB_DIAG_SERVER_NO_RESPONSE_COUNT:
        case MB_DIAG_SERVER_NAK_COUNT:
        case MB_DIAG_SERVER_BUSY_COUNT:
        case MB_DIAG_CHARACTER_OVERRUN_COUNT: {
            uint16_t value = 0;
            switch (req.subFunction) {
                case MB_DIAG_RETURN_DIAGNOSTIC_REG:    value = 0; break;
                case MB_DIAG_BUS_MESSAGE_COUNT:        value = _counters.busMessage; break;
                case MB_DIAG_BUS_COMM_ERROR_COUNT:     value = _counters.busCommError; break;
                case MB_DIAG_BUS_EXCEPTION_COUNT:      value = _counters.busExceptionError; break;
                case MB_DIAG_SERVER_MESSAGE_COUNT:     value = _counters.serverMessage; break;
                case MB_DIAG_SERVER_NO_RESPONSE_COUNT: value = _counters.serverNoResponse; break;
                case MB_DIAG_SERVER_NAK_COUNT:         value = _counters.serverNak; break;
                case MB_DIAG_SERVER_BUSY_COUNT:        value = _counters.serverBusy; break;
                case MB_DIAG_CHARACTER_OVERRUN_COUNT:  value = _counters.characterOverrun; break;
            }
            mb_packBE16(body + 2, value);
            bodyLen += 2;
            break;
        }

        default:
            return 0;   // Refused as an illegal data value by the caller
    }

    return mbpdu::encodeRawResponse(out, capacity, MB_FC_DIAGNOSTICS, body, bodyLen);
}

size_t ModbusServerEngine::reportServerId(uint8_t* out, size_t capacity) {
    // [byteCount][serverId...][runIndicator]. The server id is device specific;
    // the product code is the sensible default, since that is what a
    // commissioning tool is trying to learn.
    uint8_t body[MB_MAX_PDU];
    size_t at = 1;

    if (_identity.serverIdBytes && _identity.serverIdLength) {
        const size_t take = _identity.serverIdLength < sizeof body - 3
                                ? _identity.serverIdLength : sizeof body - 3;
        memcpy(body + at, _identity.serverIdBytes, take);
        at += take;
    } else if (_identity.productCode) {
        const size_t length = strlen(_identity.productCode);
        const size_t take = length < sizeof body - 3 ? length : sizeof body - 3;
        memcpy(body + at, _identity.productCode, take);
        at += take;
    }

    body[at++] = _identity.running ? 0xFF : 0x00;
    body[0] = (uint8_t)(at - 1);
    return mbpdu::encodeRawResponse(out, capacity, MB_FC_REPORT_SERVER_ID, body, at);
}

// =============================================================================
// DEVICE IDENTIFICATION
// =============================================================================

size_t ModbusServerEngine::deviceIdentification(const ModbusRequest& req, uint8_t* out,
                                                size_t capacity, uint8_t& refusal) {
    refusal = MB_EX_ILLEGAL_DATA_VALUE;
    if (req.meiType != MB_MEI_READ_DEVICE_ID) return 0;

    // Objects 0x00..0x02 are mandatory and 0x03..0x06 optional. A device that
    // answers with nothing is legal but useless: it is how a commissioning tool
    // tells one device from another.
    const char* objects[7] = {
        _identity.vendorName,
        _identity.productCode,
        _identity.majorMinorRevision,
        _identity.vendorUrl,
        _identity.productName,
        _identity.modelName,
        _identity.userApplicationName,
    };

    // Which category the request asks for, and therefore which objects are in
    // range: 01 basic (0x00..0x02), 02 regular (0x00..0x06), 03 extended (the
    // same here, since we define no private objects), 04 one specific object.
    uint8_t first = 0;
    uint8_t last = 2;

    switch (req.readDeviceIdCode) {
        case 0x01: first = 0; last = 2; break;   // Basic
        case 0x02: first = 0; last = 6; break;   // Regular
        case 0x03: first = 0; last = 6; break;   // Extended
        case 0x04:
            // An object this device does not have is an illegal data ADDRESS,
            // per the specification - the request itself is well formed.
            if (req.objectId > 6 || !objects[req.objectId]) {
                refusal = MB_EX_ILLEGAL_DATA_ADDRESS;
                return 0;
            }
            first = req.objectId;
            last = req.objectId;
            break;
        default:
            return 0;
    }

    // A stream read starts at the object asked for. That is how a client
    // continues after More Follows: it asks again from the Next Object Id it
    // was given. An id outside the category starts from the beginning, as the
    // specification says.
    if (req.readDeviceIdCode != 0x04 && req.objectId >= first && req.objectId <= last) {
        first = req.objectId;
    }

    // The conformity level describes what THIS DEVICE has, not what was asked
    // for. A device carrying only the three mandatory objects is basic
    // identification however it is interrogated, and claiming more would tell a
    // commissioning tool to ask for objects that are not there.
    //
    // pymodbus hardcodes 0x83 ("I support everything right now") regardless of
    // what it actually holds. That is a divergence, it is deliberate,
    // and the specification is on this side of it.
    uint8_t conformity = 0x01;
    for (uint8_t id = 3; id <= 6; id++) {
        if (objects[id]) { conformity = 0x02; break; }
    }

    // Mandatory objects must be present for a basic read to mean anything.
    if (req.readDeviceIdCode != 0x04) {
        for (uint8_t i = 0; i <= 2; i++) {
            if (!objects[i]) return 0;
        }
    }

    uint8_t body[MB_MAX_PDU];
    size_t at = 0;
    body[at++] = MB_MEI_READ_DEVICE_ID;
    body[at++] = req.readDeviceIdCode;

    // 0x80 says both stream and individual access are supported - which they
    // are, since read code 0x04 is handled above.
    body[at++] = (uint8_t)(0x80 | conformity);

    const size_t moreFollowsAt = at;
    body[at++] = 0x00;                 // More Follows, backfilled below
    const size_t nextObjectAt = at;
    body[at++] = 0x00;                 // Next Object Id, backfilled below
    const size_t countAt = at;
    body[at++] = 0x00;                 // Number Of Objects, backfilled below

    uint8_t written = 0;
    uint8_t nextObject = 0;
    bool moreFollows = false;

    for (uint8_t id = first; id <= last; id++) {
        if (!objects[id]) continue;

        const size_t length = strlen(objects[id]);
        if (length > 255) continue;

        // The response must fit one PDU. Where it does not, More Follows says
        // so and names the object to ask for next, and the client issues
        // another transaction. Truncating instead would silently lose an object
        // and give no sign that anything was missing.
        if (at + 2 + length > sizeof body || at + 2 + length + 1 > capacity) {
            moreFollows = true;
            nextObject = id;
            break;
        }

        body[at++] = id;
        body[at++] = (uint8_t)length;
        memcpy(body + at, objects[id], length);
        at += length;
        written++;
    }

    if (written == 0) return 0;

    body[moreFollowsAt] = moreFollows ? 0xFF : 0x00;
    body[nextObjectAt] = moreFollows ? nextObject : 0x00;
    body[countAt] = written;

    return mbpdu::encodeRawResponse(out, capacity, MB_FC_ENCAPSULATED_INTERFACE, body, at);
}
