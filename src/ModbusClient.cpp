#include "ModbusClient.h"

#include <string.h>
#include "ModbusNoExcept.h"

ModbusClient::ModbusClient(ModbusLink& link) noexcept : _link(link) {}

bool ModbusClient::begin(const ModbusClientConfig& config) {
    _config = config;
    _ops.setCapacity(config.queueCapacity, config.payloadSpace);
    _holdUntil = 0;
    return _ops.reserve();
}

void ModbusClient::raise(ModbusEvent event, uint8_t unit, const char* detail) {
    if (_eventHandler) _eventHandler(_eventContext, event, unit, detail);
}

// =============================================================================
// PER-UNIT TIMING
// =============================================================================

ModbusEstimator* ModbusClient::timingFor(uint8_t unit) {
    for (size_t i = 0; i < _timingCount; i++) {
        if (_timing[i].unit == unit) return &_timing[i].estimator;
    }
    size_t row = _timingCount;
    if (row < MODBUS_MAX_UNITS) {
        _timingCount++;
    } else {
        // Full - only on a board talking to more units than it keeps rows for.
        // The unit heard from longest ago gives up its row; one whose timeout
        // was set by hand never does, because that setting is not measured
        // back.
        const mb_time_t now = mb_millis();
        row = MODBUS_MAX_UNITS;
        for (size_t i = 0; i < _timingCount; i++) {
            if (_timing[i].pinnedMs != MODBUS_AUTO) continue;
            if (_timing[i].lastHeard == 0) { row = i; break; }   // never answered
            if (row == MODBUS_MAX_UNITS ||
                mb_elapsed(_timing[i].lastHeard, now) > mb_elapsed(_timing[row].lastHeard, now)) {
                row = i;
            }
        }
        if (row == MODBUS_MAX_UNITS) return nullptr;
    }
    _timing[row] = UnitTiming();
    _timing[row].unit = unit;
    return &_timing[row].estimator;
}

const ModbusEstimator* ModbusClient::timingFor(uint8_t unit) const {
    for (size_t i = 0; i < _timingCount; i++) {
        if (_timing[i].unit == unit) return &_timing[i].estimator;
    }
    return nullptr;
}

mb_time_t ModbusClient::timeoutFloorMs() const {
    // What THIS HOST can enforce. A timeout shorter than the gap between two
    // poll() calls cannot be met however fast the device answers, so an
    // application polling every 50 ms while asking for 20 would see phantom
    // timeouts for ever.
    //
    // What the MEDIUM needs is added separately, per operation, in poll() -
    // because a one register read and a 125 register read are the same request
    // and very different amounts of airtime, and one figure for both declares
    // the second failed while it is still arriving.
    mb_time_t floor = _link.serviceIntervalMs() * 2;
    if (floor < MODBUS_TIMING_FLOOR_MS) floor = MODBUS_TIMING_FLOOR_MS;
    return floor;
}

mb_time_t ModbusClient::roundTripMs(uint8_t unit) const {
    const ModbusEstimator* estimator = timingFor(unit);
    return estimator && estimator->hasSample() ? estimator->smoothed() : 0;
}

void ModbusClient::setUnitTimeout(uint8_t unit, mb_time_t timeoutMs) {
    if (timeoutMs > MODBUS_TIMING_CEILING_MS) timeoutMs = MODBUS_TIMING_CEILING_MS;
    timingFor(unit);   // the row, created if this unit is new
    for (size_t i = 0; i < _timingCount; i++) {
        if (_timing[i].unit == unit) { _timing[i].pinnedMs = timeoutMs; return; }
    }
}

mb_time_t ModbusClient::unitTimeout(uint8_t unit) const {
    for (size_t i = 0; i < _timingCount; i++) {
        if (_timing[i].unit == unit) return _timing[i].pinnedMs;
    }
    return MODBUS_AUTO;
}

mb_time_t ModbusClient::responseTimeoutMs(uint8_t unit) const {
    // A unit's own figure wins over everything: it is the one device whose
    // answer is known to take that long - a radio hop, a slow gateway.
    for (size_t i = 0; i < _timingCount; i++) {
        if (_timing[i].unit == unit && _timing[i].pinnedMs != MODBUS_AUTO) {
            return _timing[i].pinnedMs;
        }
    }

    // A pinned figure wins outright. That is what pinning means.
    if (_config.responseTimeout != MODBUS_AUTO) return _config.responseTimeout;

    mb_time_t floor = timeoutFloorMs();

    const ModbusEstimator* estimator = timingFor(unit);
    if (!estimator || !estimator->hasSample()) {
        // Nothing measured yet. Generous by default, because the cost of
        // waiting too long ONCE is a slow first transaction, and the cost of
        // waiting too little is a device declared absent that was merely
        // thinking.
        return mb_setting<mb_time_t>(_config.coldStartTimeout, floor * 10);
    }
    return estimator->timeout(floor);
}

// =============================================================================
// QUEUEING
// =============================================================================

ModbusResult ModbusClient::refuse(uint8_t unit, uint16_t address, ModbusResult why,
                                  ModbusCompletionHandler onComplete) {
    // Reported through the queue's local path rather than called here, so a
    // refusal reaches the caller from poll() exactly as a success does. Calling
    // the handler inside the call that was refused would run application code
    // before that call has returned, and a caller has not yet recorded what it
    // is waiting for.
    _ops.reportLocal(onComplete, why, unit, address);
    return why;
}

ModbusResult ModbusClient::queue(uint8_t unit, const ModbusRequest& request,
                                 const uint8_t* payload, uint16_t payloadLen,
                                 void* dest, ModbusResultKind kind,
                                 uint16_t destCapacity, uint16_t stride,
                                 ModbusCompletionHandler onComplete) {
    const bool broadcast = (unit == MB_ADDRESS_BROADCAST);

    // Refused here rather than a round trip later. A caller told immediately can
    // fix its own bug; one told after a timeout cannot tell a bad request from
    // an absent device.
    if (broadcast && !mb_isWriteFunction(request.function)) {
        return refuse(unit, request.address, ModbusResult::InvalidArgument, onComplete);
    }
    if (unit >= MB_ADDRESS_RESERVED_MIN && unit != MB_UNIT_SELF) {
        // 248..255 are reserved on a serial line. Our receive path tolerates
        // them, because real devices use them, but we will not ADDRESS one.
        return refuse(unit, request.address, ModbusResult::InvalidArgument, onComplete);
    }

    ModbusRequest checked = request;
    checked.data = payload;
    checked.dataLen = payloadLen;
    const uint8_t invalid = mbpdu::validateRequest(checked);
    if (invalid != MB_EX_NONE) {
        return refuse(unit, request.address,
                      invalid == MB_EX_ILLEGAL_DATA_ADDRESS ? ModbusResult::NotMapped
                                                            : ModbusResult::InvalidArgument,
                      onComplete);
    }

    ModbusPendingOp op;
    op.unit = unit;
    op.request = request;
    op.request.data = nullptr;          // Bound at encode time, never stored
    op.request.dataLen = payloadLen;
    op.resultPtr = dest;
    op.resultKind = kind;
    op.resultCapacity = destCapacity;
    op.resultStride = stride;
    op.queuedAt = mb_millis();
    op.broadcast = broadcast;
    op.onComplete = onComplete;

    const ModbusResult result = _ops.add(op, payload, payloadLen);
    if (result != ModbusResult::Queued) {
        // Duplicate and QueueFull are conclusions in their own right, and the
        // caller is told once, like any other.
        _ops.reportLocal(onComplete, result, unit, request.address);
    }
    return result;
}

// =============================================================================
// DRIVING
// =============================================================================

void ModbusClient::poll() {
    // What was owed for an operation refused or served in place. Before the
    // queued work, so a handler that queues something is serviced on this pass
    // rather than waiting for the next.
    _ops.dispatchLocal();

    const mb_time_t now = mb_millis();

    // Expiry is per unit, because the timeout is - and per operation, because
    // the AIRTIME is. Each operation gets its own device's measured round trip
    // plus the time the medium physically needs to carry that particular
    // exchange.
    const size_t before = _ops.pendingCount();
    uint32_t judged[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (size_t i = 0; i < _timingCount; i++) {
        const uint8_t unit = _timing[i].unit;
        _ops.expire(now, responseTimeoutMs(unit), mb_retryLimit(_config.maxRetries),
                    _link.transport().bitsPerSecond(), (int16_t)unit);
        judged[unit >> 5] |= 1u << (unit & 31);
    }
    // And once more for units nothing is known about yet - their first
    // transaction has no estimator to be judged against. ONLY those: a unit
    // already judged above against its own timeout is skipped, or a device
    // with a long one (a radio link) would be cut off at the cold-start
    // figure regardless.
    _ops.expire(now, responseTimeoutMs(0), mb_retryLimit(_config.maxRetries),
                _link.transport().bitsPerSecond(), -1, judged);
    if (_ops.pendingCount() < before) raise(ModbusEvent::RequestTimeout, 0, nullptr);

    if (!_link.isUp()) return;

    // A broadcast's turnaround, or a configured inter-frame delay. The link is
    // held rather than the operation deferred, because what is being waited for
    // is the MEDIUM, not this particular request.
    if (_holdUntil != 0) {
        if (mb_isFuture(now, _holdUntil)) return;
        _holdUntil = 0;
    }

    if (!_link.readyToSend()) return;

    ModbusPendingOp* op = _ops.nextReady(now, _config.maxInFlight,
                                         mb_setting<mb_time_t>(_config.retryDelay,
                                                               responseTimeoutMs(0) / 2));
    if (!op) return;

    uint8_t pdu[MB_MAX_PDU];
    size_t pduLen = 0;
    if (op->passthrough) {
        // Sent exactly as it arrived. Re-encoding from a decoded struct would
        // quietly drop anything the struct cannot hold, and a gateway must
        // carry function codes it does not itself implement.
        pduLen = op->payloadLen;
        if (pduLen > 0 && pduLen <= sizeof pdu) memcpy(pdu, _ops.payloadOf(*op), pduLen);
        else pduLen = 0;
    } else {
        const ModbusRequest request = _ops.bound(*op);
        pduLen = mbpdu::encodeRequest(pdu, sizeof pdu, request);
    }
    if (pduLen == 0) {
        // It will never encode, so leaving it queued would block every
        // operation behind it for good.
        const uint8_t unit = op->unit;
        _ops.cancelUnit(unit, ModbusResult::InvalidArgument);
        raise(ModbusEvent::UnexpectedResponse, unit, "operation could not be encoded");
        return;
    }

    ModbusAdu adu;
    adu.unit = op->unit;
    adu.transactionId = op->opId;
    adu.pdu = pdu;
    adu.pduLen = (uint16_t)pduLen;

    if (!_link.send(adu)) {
        // The medium refused it. Left queued so the retry path deals with it,
        // rather than being concluded as a protocol failure it was not.
        return;
    }

    if (op->broadcast) {
        // No response is coming, by design. Transmission is its conclusion, and
        // the link is then held for the turnaround: every server on the segment
        // is performing the write and none will say when it is done.
        _holdUntil = now + mb_setting<mb_time_t>(_config.turnaroundDelay,
                                                 MODBUS_TURNAROUND_MS);
        _ops.completeBroadcast(*op);
        return;
    }

    _ops.markSent(*op, now);

    if (_config.interFrameDelay) {
        _holdUntil = now + _config.interFrameDelay;
    }
}

bool ModbusClient::handleAdu(const ModbusAdu& adu) {
    const mb_time_t now = mb_millis();

    // Noted whether or not the frame turns out to answer anything of ours: a
    // unit that is transmitting is a unit that is alive, and that is what the
    // question "is that device there" is really asking.
    noteHeard(adu.unit, now);

    return _ops.applyResponse(adu.unit, adu.transactionId, adu.hasTransactionId,
                              adu.pdu, adu.pduLen, now, timingFor(adu.unit));
}

void ModbusClient::noteHeard(uint8_t unit, mb_time_t now) {
    timingFor(unit);   // Creates the row if this is the first we have heard
    for (size_t i = 0; i < _timingCount; i++) {
        if (_timing[i].unit == unit) { _timing[i].lastHeard = now; return; }
    }
}

mb_time_t ModbusClient::silenceFor(uint8_t unit) const {
    for (size_t i = 0; i < _timingCount; i++) {
        if (_timing[i].unit != unit) continue;
        if (_timing[i].lastHeard == 0) return MB_NEVER_HEARD;
        return mb_elapsed(_timing[i].lastHeard, mb_millis());
    }
    return MB_NEVER_HEARD;
}

// =============================================================================
// READS
// =============================================================================

ModbusResult ModbusClient::readCoils(uint8_t unit, uint16_t address, bool* dest,
                                     uint16_t count, ModbusCompletionHandler onComplete,
                                     uint16_t stride) {
    if (!dest) return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    return queue(unit, mbpdu::readCoils(address, count), nullptr, 0,
                 dest, ModbusResultKind::Bits, 0, stride, onComplete);
}

ModbusResult ModbusClient::readDiscreteInputs(uint8_t unit, uint16_t address, bool* dest,
                                              uint16_t count,
                                              ModbusCompletionHandler onComplete,
                                              uint16_t stride) {
    if (!dest) return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    return queue(unit, mbpdu::readDiscreteInputs(address, count), nullptr, 0,
                 dest, ModbusResultKind::Bits, 0, stride, onComplete);
}

ModbusResult ModbusClient::probeOrder(uint8_t unit, uint16_t address,
                                      ModbusOrderProbe& dest,
                                      ModbusCompletionHandler onComplete) {
    return queue(unit, mbpdu::readHoldingRegisters(address, 2), nullptr, 0,
                 &dest, ModbusResultKind::Probe, 0, 0, onComplete);
}

ModbusResult ModbusClient::readHoldingRegisters(uint8_t unit, uint16_t address,
                                                uint16_t* dest, uint16_t count,
                                                ModbusCompletionHandler onComplete,
                                                uint16_t stride) {
    if (!dest) return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    return queue(unit, mbpdu::readHoldingRegisters(address, count), nullptr, 0,
                 dest, ModbusResultKind::Registers, 0, stride, onComplete);
}

ModbusResult ModbusClient::readInputRegisters(uint8_t unit, uint16_t address,
                                              uint16_t* dest, uint16_t count,
                                              ModbusCompletionHandler onComplete,
                                              uint16_t stride) {
    if (!dest) return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    return queue(unit, mbpdu::readInputRegisters(address, count), nullptr, 0,
                 dest, ModbusResultKind::Registers, 0, stride, onComplete);
}

ModbusResult ModbusClient::readCoil(uint8_t unit, uint16_t address, bool& dest,
                                    ModbusCompletionHandler onComplete) {
    return readCoils(unit, address, &dest, 1, onComplete);
}

ModbusResult ModbusClient::readDiscreteInput(uint8_t unit, uint16_t address, bool& dest,
                                             ModbusCompletionHandler onComplete) {
    return readDiscreteInputs(unit, address, &dest, 1, onComplete);
}

ModbusResult ModbusClient::readHoldingRegister(uint8_t unit, uint16_t address,
                                               uint16_t& dest,
                                               ModbusCompletionHandler onComplete) {
    return readHoldingRegisters(unit, address, &dest, 1, onComplete);
}

ModbusResult ModbusClient::readInputRegister(uint8_t unit, uint16_t address, uint16_t& dest,
                                             ModbusCompletionHandler onComplete) {
    return readInputRegisters(unit, address, &dest, 1, onComplete);
}

// =============================================================================
// WRITES
// =============================================================================

ModbusResult ModbusClient::writeCoil(uint8_t unit, uint16_t address, bool value,
                                     ModbusCompletionHandler onComplete) {
    return queue(unit, mbpdu::writeSingleCoil(address, value), nullptr, 0,
                 nullptr, ModbusResultKind::None, 0, 0, onComplete);
}

ModbusResult ModbusClient::writeRegister(uint8_t unit, uint16_t address, uint16_t value,
                                         ModbusCompletionHandler onComplete) {
    return queue(unit, mbpdu::writeSingleRegister(address, value), nullptr, 0,
                 nullptr, ModbusResultKind::None, 0, 0, onComplete);
}

ModbusResult ModbusClient::writeCoils(uint8_t unit, uint16_t address, const bool* values,
                                      uint16_t count, ModbusCompletionHandler onComplete,
                                      uint16_t stride) {
    if (!values || count == 0) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }
    if (count > MB_MAX_WRITE_BITS) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }

    // Packed into an owned buffer HERE, while the caller's array is known to be
    // alive. Holding a pointer into it until the bus gets round to the write
    // would be a dangling pointer the moment the caller's array went out of
    // scope - which, for a local, is the next line.
    uint8_t packed[MB_MAX_WRITE_BITS / 8 + 1];
    const uint16_t bytes = mb_packedByteCount(count);
    memset(packed, 0, bytes);
    for (uint16_t i = 0; i < count; i++) {
        if (*mb_elemAt<bool>(values, i, stride)) mb_setPackedBit(packed, i, true);
    }

    return queue(unit, mbpdu::writeMultipleCoils(address, count, nullptr),
                 packed, bytes, nullptr, ModbusResultKind::None, 0, 0, onComplete);
}

ModbusResult ModbusClient::writeRegisters(uint8_t unit, uint16_t address,
                                          const uint16_t* values, uint16_t count,
                                          ModbusCompletionHandler onComplete,
                                          uint16_t stride) {
    if (!values || count == 0) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }
    if (count > MB_MAX_WRITE_REGISTERS) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }

    uint8_t payload[MB_MAX_WRITE_REGISTERS * 2];
    for (uint16_t i = 0; i < count; i++) {
        mb_packBE16(payload + (size_t)i * 2, *mb_elemAt<uint16_t>(values, i, stride));
    }

    return queue(unit, mbpdu::writeMultipleRegisters(address, count, nullptr),
                 payload, (uint16_t)(count * 2), nullptr, ModbusResultKind::None,
                 0, 0, onComplete);
}

ModbusResult ModbusClient::maskWriteRegister(uint8_t unit, uint16_t address,
                                             uint16_t andMask, uint16_t orMask,
                                             ModbusCompletionHandler onComplete) {
    return queue(unit, mbpdu::maskWriteRegister(address, andMask, orMask), nullptr, 0,
                 nullptr, ModbusResultKind::None, 0, 0, onComplete);
}

ModbusResult ModbusClient::readWriteRegisters(uint8_t unit,
                                              uint16_t readAddress, uint16_t* dest,
                                              uint16_t readCount,
                                              uint16_t writeAddress,
                                              const uint16_t* values, uint16_t writeCount,
                                              ModbusCompletionHandler onComplete) {
    if (!dest || !values || readCount == 0 || writeCount == 0) {
        return refuse(unit, readAddress, ModbusResult::InvalidArgument, onComplete);
    }
    if (writeCount > MB_MAX_RW_WRITE_REGISTERS) {
        return refuse(unit, readAddress, ModbusResult::InvalidArgument, onComplete);
    }

    uint8_t payload[MB_MAX_RW_WRITE_REGISTERS * 2];
    for (uint16_t i = 0; i < writeCount; i++) {
        mb_packBE16(payload + (size_t)i * 2, values[i]);
    }

    return queue(unit,
                 mbpdu::readWriteMultipleRegisters(readAddress, readCount,
                                                   writeAddress, writeCount, nullptr),
                 payload, (uint16_t)(writeCount * 2),
                 dest, ModbusResultKind::Registers, 0, 0, onComplete);
}

// =============================================================================
// DIAGNOSTIC AND IDENTITY FUNCTIONS
// =============================================================================

ModbusResult ModbusClient::readExceptionStatus(uint8_t unit, uint8_t& dest,
                                               ModbusCompletionHandler onComplete) {
    return queue(unit, mbpdu::readExceptionStatus(), nullptr, 0,
                 &dest, ModbusResultKind::Raw, 1, 0, onComplete);
}

ModbusResult ModbusClient::diagnostics(uint8_t unit, uint16_t subFunction,
                                       const uint8_t* data, uint16_t dataLen,
                                       uint8_t* dest, uint16_t destCapacity,
                                       ModbusCompletionHandler onComplete) {
    return queue(unit, mbpdu::diagnostics(subFunction, nullptr, dataLen),
                 data, dataLen, dest, dest ? ModbusResultKind::Raw : ModbusResultKind::None,
                 destCapacity, 0, onComplete);
}

ModbusResult ModbusClient::reportServerId(uint8_t unit, uint8_t* dest,
                                          uint16_t destCapacity,
                                          ModbusCompletionHandler onComplete) {
    return queue(unit, mbpdu::reportServerId(), nullptr, 0,
                 dest, dest ? ModbusResultKind::Raw : ModbusResultKind::None,
                 destCapacity, 0, onComplete);
}

ModbusResult ModbusClient::readDeviceIdentification(uint8_t unit, uint8_t readDeviceIdCode,
                                                    uint8_t objectId,
                                                    uint8_t* dest, uint16_t destCapacity,
                                                    ModbusCompletionHandler onComplete) {
    return queue(unit, mbpdu::readDeviceIdentification(readDeviceIdCode, objectId),
                 nullptr, 0, dest,
                 dest ? ModbusResultKind::Raw : ModbusResultKind::None,
                 destCapacity, 0, onComplete);
}

ModbusResult ModbusClient::readFifoQueue(uint8_t unit, uint16_t pointerAddress,
                                         uint16_t* dest, uint16_t maxValues,
                                         ModbusCompletionHandler onComplete) {
    // Raw rather than Registers: a FIFO response says how many values it
    // carries, where a register read's count comes from the request. Unpacking
    // it as registers would write `maxValues` entries whatever arrived.
    return queue(unit, mbpdu::readFifoQueue(pointerAddress), nullptr, 0,
                 dest, dest ? ModbusResultKind::Fifo : ModbusResultKind::None,
                 maxValues, 0, onComplete);
}

// =============================================================================
// VALUES WIDER THAN A REGISTER
// =============================================================================

ModbusResult ModbusClient::queueWide(uint8_t unit, uint16_t address, void* dest,
                                     ModbusWideType type, ModbusOrder order,
                                     uint16_t count, uint16_t stride,
                                     ModbusCompletionHandler onComplete,
                                     ModbusTable table) {
    if (!dest || count == 0) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }

    const uint16_t perValue = mb_wideRegisters(type);
    const uint32_t registers = (uint32_t)count * perValue;
    if (registers == 0 || registers > MB_MAX_READ_REGISTERS) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }

    // A bit list cannot carry a 32-bit value, and asking it to would send a
    // read-coils request whose response is then decoded as registers.
    if (table != ModbusTable::HoldingRegister && table != ModbusTable::InputRegister) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }

    ModbusPendingOp op;
    op.unit = unit;
    // Devices publish totals and flow rates in the input register list at
    // least as often as the holding list, and a wide read that could only
    // reach one of them would send the caller back to raw registers.
    op.request = (table == ModbusTable::InputRegister)
                     ? mbpdu::readInputRegisters(address, (uint16_t)registers)
                     : mbpdu::readHoldingRegisters(address, (uint16_t)registers);
    op.request.data = nullptr;
    op.resultPtr = dest;
    op.resultKind = ModbusResultKind::Typed;
    op.resultStride = stride;
    op.wideType = type;
    op.wideOrder = order;
    op.wideCount = count;
    op.queuedAt = mb_millis();
    op.onComplete = onComplete;

    const ModbusResult result = _ops.add(op);
    if (result != ModbusResult::Queued) {
        _ops.reportLocal(onComplete, result, unit, address);
    }
    return result;
}

ModbusResult ModbusClient::writeWide(uint8_t unit, uint16_t address, const uint8_t* wire,
                                     uint16_t registers,
                                     ModbusCompletionHandler onComplete) {
    // Converted to registers HERE, while the caller's value is known to be
    // alive, and copied into the operation. Nothing points back at the caller.
    uint16_t values[4];
    mb_wireToRegisters(wire, registers, values);
    return writeRegisters(unit, address, values, registers, onComplete);
}

ModbusResult ModbusClient::readFloat32(uint8_t unit, uint16_t address, float& dest,
                                       ModbusOrder order,
                                       ModbusCompletionHandler onComplete,
                                       ModbusTable table) {
    return queueWide(unit, address, &dest, ModbusWideType::Float32, order, 1, 0, onComplete,
                     table);
}

ModbusResult ModbusClient::readInt32(uint8_t unit, uint16_t address, int32_t& dest,
                                     ModbusOrder order,
                                     ModbusCompletionHandler onComplete,
                                     ModbusTable table) {
    return queueWide(unit, address, &dest, ModbusWideType::Int32, order, 1, 0, onComplete,
                     table);
}

ModbusResult ModbusClient::readUInt32(uint8_t unit, uint16_t address, uint32_t& dest,
                                      ModbusOrder order,
                                      ModbusCompletionHandler onComplete,
                                      ModbusTable table) {
    return queueWide(unit, address, &dest, ModbusWideType::UInt32, order, 1, 0, onComplete,
                     table);
}

ModbusResult ModbusClient::readFloat64(uint8_t unit, uint16_t address, double& dest,
                                       ModbusOrder order,
                                       ModbusCompletionHandler onComplete,
                                       ModbusTable table) {
    return queueWide(unit, address, &dest, ModbusWideType::Float64, order, 1, 0, onComplete,
                     table);
}

ModbusResult ModbusClient::readInt64(uint8_t unit, uint16_t address, int64_t& dest,
                                     ModbusOrder order,
                                     ModbusCompletionHandler onComplete,
                                     ModbusTable table) {
    return queueWide(unit, address, &dest, ModbusWideType::Int64, order, 1, 0, onComplete,
                     table);
}

ModbusResult ModbusClient::readFloat32Array(uint8_t unit, uint16_t address, float* dest,
                                            uint16_t count, ModbusOrder order,
                                            ModbusCompletionHandler onComplete,
                                            uint16_t stride,
                                            ModbusTable table) {
    return queueWide(unit, address, dest, ModbusWideType::Float32, order, count,
                     stride, onComplete, table);
}

ModbusResult ModbusClient::readInt32Array(uint8_t unit, uint16_t address, int32_t* dest,
                                          uint16_t count, ModbusOrder order,
                                          ModbusCompletionHandler onComplete,
                                          uint16_t stride,
                                          ModbusTable table) {
    return queueWide(unit, address, dest, ModbusWideType::Int32, order, count,
                     stride, onComplete, table);
}

ModbusResult ModbusClient::readUInt32Array(uint8_t unit, uint16_t address, uint32_t* dest,
                                           uint16_t count, ModbusOrder order,
                                           ModbusCompletionHandler onComplete,
                                           uint16_t stride,
                                           ModbusTable table) {
    return queueWide(unit, address, dest, ModbusWideType::UInt32, order, count,
                     stride, onComplete, table);
}

ModbusResult ModbusClient::readFloat64Array(uint8_t unit, uint16_t address, double* dest,
                                            uint16_t count, ModbusOrder order,
                                            ModbusCompletionHandler onComplete,
                                            uint16_t stride,
                                            ModbusTable table) {
    return queueWide(unit, address, dest, ModbusWideType::Float64, order, count,
                     stride, onComplete, table);
}

ModbusResult ModbusClient::readInt64Array(uint8_t unit, uint16_t address, int64_t* dest,
                                          uint16_t count, ModbusOrder order,
                                          ModbusCompletionHandler onComplete,
                                          uint16_t stride,
                                          ModbusTable table) {
    return queueWide(unit, address, dest, ModbusWideType::Int64, order, count,
                     stride, onComplete, table);
}

ModbusResult ModbusClient::readWideArray(uint8_t unit, uint16_t address, void* dest,
                                         ModbusWideType type, ModbusOrder order,
                                         uint16_t count, uint16_t stride,
                                         ModbusCompletionHandler onComplete,
                                         ModbusTable table) {
    return queueWide(unit, address, dest, type, order, count, stride, onComplete, table);
}

ModbusResult ModbusClient::writeWideArray(uint8_t unit, uint16_t address, const void* values,
                                          ModbusWideType type, ModbusOrder order,
                                          uint16_t count, uint16_t stride,
                                          ModbusCompletionHandler onComplete) {
    const uint16_t perValue = mb_wideRegisters(type);
    if (!values || count == 0 || perValue == 0) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }

    // Refused here rather than on the wire. A caller that hands over an array
    // of forty float64s is asking for 160 registers, and finding that out from
    // a device's exception - if the device bothers to raise one - is finding
    // it out in the wrong place.
    const uint32_t registers = (uint32_t)count * perValue;
    if (registers > MB_MAX_WRITE_REGISTERS) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }

    // Packed HERE, while the caller's array is known to be alive. The
    // operation then carries registers and nothing points back at the caller.
    uint16_t packed[MB_MAX_WRITE_REGISTERS];
    const uint16_t width = (uint16_t)(perValue * 2);
    const uint16_t step = stride ? stride : width;

    for (uint16_t i = 0; i < count; i++) {
        const uint8_t* element = (const uint8_t*)values + (size_t)i * step;
        uint8_t wire[8];

        switch (type) {
            case ModbusWideType::Float32: {
                float v;
                memcpy(&v, element, sizeof(v));
                mb_fromFloat32(v, wire, order);
                break;
            }
            case ModbusWideType::Int32: {
                int32_t v;
                memcpy(&v, element, sizeof(v));
                mb_fromInt32(v, wire, order);
                break;
            }
            case ModbusWideType::UInt32: {
                uint32_t v;
                memcpy(&v, element, sizeof(v));
                mb_fromUInt32(v, wire, order);
                break;
            }
            case ModbusWideType::Float64: {
                double v;
                memcpy(&v, element, sizeof(v));
                mb_fromFloat64(v, wire, order);
                break;
            }
            case ModbusWideType::Int64: {
                int64_t v;
                memcpy(&v, element, sizeof(v));
                mb_fromInt64(v, wire, order);
                break;
            }
            case ModbusWideType::UInt64: {
                uint64_t v;
                memcpy(&v, element, sizeof(v));
                mb_fromUInt64(v, wire, order);
                break;
            }
            case ModbusWideType::None:
                return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
        }

        mb_wireToRegisters(wire, perValue, packed + (size_t)i * perValue);
    }

    return writeRegisters(unit, address, packed, (uint16_t)registers, onComplete);
}

ModbusResult ModbusClient::writeFloat32Array(uint8_t unit, uint16_t address,
                                             const float* values, uint16_t count,
                                             ModbusOrder order,
                                             ModbusCompletionHandler onComplete,
                                             uint16_t stride) {
    return writeWideArray(unit, address, values, ModbusWideType::Float32, order,
                          count, stride, onComplete);
}

ModbusResult ModbusClient::writeInt32Array(uint8_t unit, uint16_t address,
                                           const int32_t* values, uint16_t count,
                                           ModbusOrder order,
                                           ModbusCompletionHandler onComplete,
                                           uint16_t stride) {
    return writeWideArray(unit, address, values, ModbusWideType::Int32, order,
                          count, stride, onComplete);
}

ModbusResult ModbusClient::writeUInt32Array(uint8_t unit, uint16_t address,
                                            const uint32_t* values, uint16_t count,
                                            ModbusOrder order,
                                            ModbusCompletionHandler onComplete,
                                            uint16_t stride) {
    return writeWideArray(unit, address, values, ModbusWideType::UInt32, order,
                          count, stride, onComplete);
}

ModbusResult ModbusClient::writeFloat64Array(uint8_t unit, uint16_t address,
                                             const double* values, uint16_t count,
                                             ModbusOrder order,
                                             ModbusCompletionHandler onComplete,
                                             uint16_t stride) {
    return writeWideArray(unit, address, values, ModbusWideType::Float64, order,
                          count, stride, onComplete);
}

ModbusResult ModbusClient::writeInt64Array(uint8_t unit, uint16_t address,
                                           const int64_t* values, uint16_t count,
                                           ModbusOrder order,
                                           ModbusCompletionHandler onComplete,
                                           uint16_t stride) {
    return writeWideArray(unit, address, values, ModbusWideType::Int64, order,
                          count, stride, onComplete);
}

ModbusResult ModbusClient::writeFloat32(uint8_t unit, uint16_t address, float value,
                                        ModbusOrder order,
                                        ModbusCompletionHandler onComplete) {
    uint8_t wire[4];
    mb_fromFloat32(value, wire, order);
    return writeWide(unit, address, wire, 2, onComplete);
}

ModbusResult ModbusClient::writeInt32(uint8_t unit, uint16_t address, int32_t value,
                                      ModbusOrder order,
                                      ModbusCompletionHandler onComplete) {
    uint8_t wire[4];
    mb_fromInt32(value, wire, order);
    return writeWide(unit, address, wire, 2, onComplete);
}

ModbusResult ModbusClient::writeUInt32(uint8_t unit, uint16_t address, uint32_t value,
                                       ModbusOrder order,
                                       ModbusCompletionHandler onComplete) {
    uint8_t wire[4];
    mb_fromUInt32(value, wire, order);
    return writeWide(unit, address, wire, 2, onComplete);
}

ModbusResult ModbusClient::writeFloat64(uint8_t unit, uint16_t address, double value,
                                        ModbusOrder order,
                                        ModbusCompletionHandler onComplete) {
    uint8_t wire[8];
    mb_fromFloat64(value, wire, order);
    return writeWide(unit, address, wire, 4, onComplete);
}

ModbusResult ModbusClient::writeInt64(uint8_t unit, uint16_t address, int64_t value,
                                      ModbusOrder order,
                                      ModbusCompletionHandler onComplete) {
    uint8_t wire[8];
    mb_fromInt64(value, wire, order);
    return writeWide(unit, address, wire, 4, onComplete);
}

ModbusResult ModbusClient::readText(uint8_t unit, uint16_t address, char* dest,
                                    uint16_t capacity, uint16_t registers,
                                    ModbusCompletionHandler onComplete) {
    if (!dest || capacity == 0 || registers == 0) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }
    // Text is unpacked in BYTES, bounded by the caller's capacity - never in
    // registers. A response's byte count need not be even, so a bound expressed
    // in registers is half a byte short of what gets copied, and that half byte
    // is how a peer overruns the caller's buffer.
    return queue(unit, mbpdu::readHoldingRegisters(address, registers), nullptr, 0,
                 dest, ModbusResultKind::Text, capacity, 0, onComplete);
}

ModbusResult ModbusClient::writeText(uint8_t unit, uint16_t address, const char* text,
                                     uint16_t registers,
                                     ModbusCompletionHandler onComplete) {
    if (!text || registers == 0 || registers > MB_MAX_WRITE_REGISTERS) {
        return refuse(unit, address, ModbusResult::InvalidArgument, onComplete);
    }

    // Packed here, and the whole span is written even where the text is
    // shorter - so a shorter message never leaves the tail of a longer one
    // behind it, which on a fault string reads as a fault that is still there.
    uint16_t values[MB_MAX_WRITE_REGISTERS];
    memset(values, 0, (size_t)registers * sizeof values[0]);
    mb_textToRegisters(text, values, registers);
    return writeRegisters(unit, address, values, registers, onComplete);
}

// =============================================================================
// PASSTHROUGH
// =============================================================================

ModbusResult ModbusClient::forward(uint8_t unit, const uint8_t* pdu, uint16_t pduLen,
                                   ModbusPassthroughHandler onResponse) {
    if (!pdu || pduLen == 0 || pduLen > MB_MAX_PDU) {
        if (onResponse) {
            // Reported through the queue's local path, so a refusal reaches the
            // caller from poll() exactly as an answer does.
            _ops.reportLocal(onResponse, ModbusResult::InvalidArgument, unit, 0);
        }
        return ModbusResult::InvalidArgument;
    }

    ModbusPendingOp op;
    op.unit = unit;
    op.passthrough = true;
    op.request.function = pdu[0];
    op.request.data = nullptr;
    op.queuedAt = mb_millis();
    op.broadcast = (unit == MB_ADDRESS_BROADCAST);
    op.onPassthrough = onResponse;

    const ModbusResult result = _ops.add(op, pdu, pduLen);
    if (result != ModbusResult::Queued && onResponse) {
        _ops.reportLocal(onResponse, result, unit, 0);
    }
    return result;
}
