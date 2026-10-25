#include "ModbusOperations.h"

#include <new>
#include <string.h>
#include "ModbusNoExcept.h"

// =============================================================================
// STORAGE
// =============================================================================

ModbusOperations::~ModbusOperations() {
    delete[] _slots;
    delete[] _inUse;
    delete[] _order;
    delete[] _bytes;
    delete[] _deferred.items;
    delete[] _local.items;
}

void ModbusOperations::setCapacity(size_t capacity, size_t payloadSpace) {
    _capacity = capacity ? capacity : 1;
    if (_capacity > 0xFFFF) _capacity = 0xFFFF;
    _payloadSpace = payloadSpace > 0xFFFF ? 0xFFFF : payloadSpace;
}

bool ModbusOperations::reserve() {
    if (_allocated >= _capacity && _bytesAllocated >= _payloadSpace) return true;

    // Raised limits take effect once nothing is queued or owed, so nothing in
    // flight is ever moved. Until then the storage already there serves.
    if (_slots && (_count || _deferred.count || _local.count)) return true;

    delete[] _slots; delete[] _inUse; delete[] _order; delete[] _bytes;
    delete[] _deferred.items; delete[] _local.items;
    const size_t ring = _capacity + 8;
    _slots = new (std::nothrow) ModbusPendingOp[_capacity];
    _inUse = new (std::nothrow) bool[_capacity]();
    _order = new (std::nothrow) uint16_t[_capacity];
    _bytes = new (std::nothrow) uint8_t[_payloadSpace ? _payloadSpace : 1];
    _deferred.items = new (std::nothrow) Owed[ring];
    _local.items = new (std::nothrow) Owed[ring];
    _deferred.size = _local.size = (uint16_t)ring;
    _deferred.head = _local.head = 0;
    const bool ok = _slots && _inUse && _order && _bytes && _deferred.items && _local.items;
    _allocated = ok ? _capacity : 0;
    _bytesAllocated = ok ? _payloadSpace : 0;
    if (!ok) _deferred.size = _local.size = 0;
    return ok;
}

bool ModbusOperations::setPayload(ModbusPendingOp& op, const uint8_t* payload,
                                  uint16_t payloadLen) {
    // Checked before anything moves, so a refusal leaves the record as it was.
    if (_bytesUsed - op.payloadLen + payloadLen > byteRoom()) return false;
    dropPayload(op);
    op.payloadAt = (uint16_t)_bytesUsed;
    op.payloadLen = payloadLen;
    if (payloadLen) memcpy(_bytes + _bytesUsed, payload, payloadLen);
    _bytesUsed += payloadLen;
    return true;
}

void ModbusOperations::dropPayload(ModbusPendingOp& op) {
    // The area is kept packed: everything after this payload moves down, so
    // there are never holes to fit a request into.
    const size_t from = op.payloadAt, length = op.payloadLen;
    op.payloadLen = 0;
    op.payloadAt = 0;
    if (!length) return;
    memmove(_bytes + from, _bytes + from + length, _bytesUsed - from - length);
    _bytesUsed -= length;
    for (size_t i = 0; i < _count; i++) {
        ModbusPendingOp& other = at(i);
        if (other.payloadLen && other.payloadAt > from) {
            other.payloadAt = (uint16_t)(other.payloadAt - length);
        }
    }
}

bool ModbusOperations::insert(const ModbusPendingOp& op, const uint8_t* payload,
                              uint16_t payloadLen) {
    if (_bytesUsed + payloadLen > byteRoom()) return false;
    size_t slot = 0;
    while (_inUse[slot]) slot++;          // _count < _capacity, so one is free
    ModbusPendingOp& queued = _slots[slot];
    queued = op;
    queued.opId = nextOpId();
    queued.request.data = nullptr;         // Bound at encode time, never stored
    queued.request.dataLen = payloadLen;
    queued.payloadLen = 0;
    setPayload(queued, payload, payloadLen);
    _inUse[slot] = true;
    _order[_count++] = (uint16_t)slot;
    _issued++;
    return true;
}

void ModbusOperations::removeAt(size_t index) {
    const uint16_t slot = _order[index];
    dropPayload(_slots[slot]);
    _slots[slot] = ModbusPendingOp();      // lets go of any handler still held
    _inUse[slot] = false;
    memmove(_order + index, _order + index + 1, (_count - index - 1) * sizeof _order[0]);
    _count--;
}

size_t ModbusOperations::positionOf(const ModbusPendingOp* op) const {
    for (size_t i = 0; i < _count; i++) {
        if (&at(i) == op) return i;
    }
    return NO_OP;
}

bool ModbusOperations::OwedRing::push(Owed&& owed) {
    if (count >= size) return false;
    items[(head + count) % size] = std::move(owed);
    count++;
    return true;
}

ModbusOperations::Owed ModbusOperations::OwedRing::pop() {
    Owed owed = std::move(items[head]);
    head = (uint16_t)((head + 1) % size);
    count--;
    return owed;
}

// =============================================================================
// QUEUEING
// =============================================================================

ModbusResult ModbusOperations::add(const ModbusPendingOp& op, const uint8_t* payload,
                                   uint16_t payloadLen) {
    if (_count >= _capacity || !reserve() || _count >= _allocated) {
        return ModbusResult::QueueFull;
    }

    // A passthrough is never deduplicated or superseded. Two identical
    // requests arriving from two different clients are two different questions,
    // and each is owed its own answer on its own socket.
    if (op.passthrough) {
        return insert(op, payload, payloadLen) ? ModbusResult::Queued : ModbusResult::QueueFull;
    }

    for (size_t i = 0; i < _count; i++) {
        ModbusPendingOp& existing = at(i);

        if (existing.unit != op.unit ||
            existing.request.function != op.request.function ||
            existing.request.address != op.request.address ||
            existing.request.quantity != op.request.quantity) {
            continue;
        }

        const bool isWrite = mb_isWriteFunction(op.request.function);

        if (!isWrite) {
            // The same read twice is genuinely redundant: the second would
            // return exactly what the first is already fetching.
            return ModbusResult::Duplicate;
        }

        // A write that matches one still WAITING TO GO OUT supersedes it. The
        // application is saying what the register should be, not queuing a
        // history of what it should have been, and a control loop updating a
        // setpoint faster than the bus rotates does this constantly. Rejecting
        // it would leave the register holding a value the application has
        // already moved on from.
        if (existing.awaitingResponse) {
            // Already on the wire. Rewriting its payload now would leave the
            // echo, when it arrives, confirming a value that was never
            // transmitted. Let the new write queue separately behind it.
            break;
        }

        if (existing.payloadLen == payloadLen &&
            (payloadLen == 0 || memcmp(payloadOf(existing), payload, payloadLen) == 0) &&
            existing.request.value == op.request.value &&
            existing.request.andMask == op.request.andMask &&
            existing.request.orMask == op.request.orMask) {
            return ModbusResult::Duplicate;   // Same value, nothing to do
        }

        // The queued value never reaches the wire, so anything waiting on it is
        // TOLD rather than left outstanding for ever.
        //
        // Every field the new operation carries, not just its payload. The
        // completion handler belongs to the caller that asked for it, so a
        // write replacing another must not inherit the old one's terms.
        //
        // The bytes first: if the new value does not fit, the queued write is
        // left exactly as it was and the new one is refused.
        if (!setPayload(existing, payload, payloadLen)) return ModbusResult::QueueFull;

        // Taken FIRST, before the handler runs: it is application code and may
        // queue or conclude work of its own.
        ModbusCompletionHandler superseded = std::move(existing.onComplete);

        existing.request = op.request;
        existing.request.data = nullptr;
        existing.request.dataLen = payloadLen;
        existing.queuedAt = op.queuedAt;
        existing.retryCount = 0;
        existing.onComplete = op.onComplete;
        existing.opId = nextOpId();

        // The record is fully written, so nothing here reads it again and the
        // handler is free to queue.
        dispatch(Owed{ std::move(superseded), ModbusResult::Superseded });
        return ModbusResult::Queued;
    }

    if (!insert(op, payload, payloadLen)) return ModbusResult::QueueFull;
    dropFinished(op.unit, op.request.address);
    return ModbusResult::Queued;
}

void ModbusOperations::reportLocal(const ModbusCompletionHandler& handler,
                                   ModbusResult result, uint8_t unit, uint16_t address) {
    noteFinished(unit, address, result);
    if (!handler) return;
    // Held for the next poll. Only if the ring is full - a caller refused more
    // times between two polls than the queue holds - is it told at once.
    if (!reserve() || !_local.push(Owed{ handler, result })) handler(result);
}

void ModbusOperations::reportLocal(const ModbusPassthroughHandler& handler,
                                   ModbusResult result, uint8_t unit, uint16_t address) {
    noteFinished(unit, address, result);
    if (!handler) return;
    for (size_t i = 0; i < kHeldPassthroughs && reserve(); i++) {
        if (_held[i].used) continue;
        _held[i].pass = handler;
        _held[i].used = true;
        if (_local.push(Owed{ ModbusCompletionHandler(&deliverHeld, this, i), result })) return;
        _held[i].pass = nullptr;
        _held[i].used = false;
        break;
    }
    handler(result, nullptr, 0);
}

void ModbusOperations::deliverHeld(void* self, uintptr_t index, ModbusResult result) {
    HeldPassthrough& held = static_cast<ModbusOperations*>(self)->_held[index];
    const ModbusPassthroughHandler pass = std::move(held.pass);
    held.pass = nullptr;
    held.used = false;
    if (pass) pass(result, nullptr, 0);
}

void ModbusOperations::dispatchLocal() {
    // Conclusions a previous dispatch could not finish inside its bound. Owed
    // to somebody, so they are run here, on the next poll, at depth zero - not
    // left in the vector for a future conclusion to happen to notice.
    drainCarriedDeferrals();

    // Only those owed now: a handler may queue work that is refused, and that
    // conclusion is owed on the next call, as it always was.
    for (size_t owed = _local.count; owed > 0 && _local.count; owed--) {
        dispatch(_local.pop());
    }
}

// =============================================================================
// DRIVING
// =============================================================================

size_t ModbusOperations::inFlightCount() const {
    size_t n = 0;
    for (size_t i = 0; i < _count; i++) {
        if (at(i).awaitingResponse) n++;
    }
    return n;
}

size_t ModbusOperations::indexInFlight(uint8_t unit) const {
    for (size_t i = 0; i < _count; i++) {
        if (at(i).awaitingResponse && at(i).unit == unit) return i;
    }
    return NO_OP;
}

ModbusPendingOp* ModbusOperations::nextReady(mb_time_t now, size_t maxInFlight,
                                             mb_time_t retryDelayMs) {
    if (inFlightCount() >= maxInFlight) return nullptr;

    for (size_t i = 0; i < _count; i++) {
        ModbusPendingOp& op = at(i);
        if (op.awaitingResponse) continue;

        // A retry waits out its backoff. Going again on the very next poll
        // repeats the attempt into whatever swallowed the first one, and two
        // masters backing off deterministically stay in step - which is what
        // the jitter is for.
        if (op.retryCount > 0) {
            const mb_time_t jitter =
                mb_spread(op.opId ^ (uint32_t)op.retryCount,
                          retryDelayMs * MODBUS_RETRY_SPREAD_PERCENT / 100);
            if (mb_elapsed(op.queuedAt, now) < retryDelayMs + jitter) continue;
        }

        return &op;
    }
    return nullptr;
}

void ModbusOperations::markSent(ModbusPendingOp& op, mb_time_t now) {
    op.awaitingResponse = true;
    op.sentAt = now;
    op.queuedAt = now;   // The timeout runs from transmission, not from queueing
}

void ModbusOperations::completeBroadcast(ModbusPendingOp& op) {
    // A broadcast is written to every server and answered by none, so
    // transmission IS its conclusion. Reported as Ok because it is as done as
    // it will ever be; nothing further can be learnt about it, and saying
    // Queued would leave the caller waiting for an answer that the protocol
    // forbids.
    const size_t i = positionOf(&op);
    if (i != NO_OP) retireAt(i, ModbusResult::Ok);
}

// =============================================================================
// RESPONSES
// =============================================================================

size_t ModbusOperations::indexByOpId(uint8_t unit, uint16_t opId) const {
    for (size_t i = 0; i < _count; i++) {
        if (at(i).unit == unit && at(i).opId == opId && at(i).awaitingResponse) {
            return i;
        }
    }
    return NO_OP;
}

bool ModbusOperations::applyResponse(uint8_t unit, uint16_t transactionId,
                                     bool hasTransactionId,
                                     const uint8_t* pdu, size_t pduLen, mb_time_t now,
                                     ModbusEstimator* roundTrip) {
    // Matched by transaction identifier where the ADU carries one, and by "the
    // single operation in flight to this unit" where it does not. On serial
    // there is nothing else available, which is precisely why only one may be
    // outstanding at a time.
    //
    // The DIFFERENCE between the two matters below. A transaction identifier is
    // proof: this frame answers this request and no other. A positional match
    // is a guess that is almost always right, and on a shared serial bus the
    // frame could equally be another master's traffic.
    bool definitive = false;
    size_t index = NO_OP;

    if (hasTransactionId) {
        // The identifier IS the match. A response carrying one that matches
        // nothing outstanding belongs to nobody: it answers a request that has
        // already timed out, or it is a device confused about what it was
        // asked. Falling back to "the request we sent last" would accept it as
        // an answer to a different question and hand the application one
        // register's value under another's name - which is the exact failure
        // the identifier exists to prevent.
        index = indexByOpId(unit, transactionId);
        if (index == NO_OP) return false;
        definitive = true;
    } else {
        // No identifier in the framing, so position is all there is - and that
        // is why only one request may be outstanding at a time on a serial
        // line.
        index = indexInFlight(unit);
        if (index == NO_OP) return false;
    }

    // A passthrough operation is handed back verbatim. Its PDU was never
    // decoded on the way out, so there is no request shape to decode the
    // response against - only the function code can be checked, which is all
    // that can be said about a PDU whose shape is unknown.
    if (at(index).passthrough) {
        if (pduLen < 1) return false;

        const uint8_t sent = at(index).payloadLen ? payloadOf(at(index))[0] : 0;
        const uint8_t got = pdu[0];
        if (got != sent && got != mb_exceptionFunction(sent)) {
            if (definitive) {
                retireAt(index, ModbusResult::ProtocolError);
                return true;
            }
            return false;
        }

        if (roundTrip) roundTrip->addSample(mb_elapsed(at(index).sentAt, now));

        // Delivered BEFORE the operation is retired, because retiring runs the
        // completion handler and a gateway's handler will want the bytes.
        ModbusPassthroughHandler deliver = std::move(at(index).onPassthrough);
        const bool isException = mb_isExceptionFunction(got);
        const ModbusResult result =
            isException && pduLen >= 2 ? mb_exceptionToResult(pdu[1])
                                       : ModbusResult::Ok;

        // Cleared BEFORE retiring, so retireAt's own "this never got a
        // response" path does not deliver a SECOND time.
        //
        // It would, otherwise, for every exception: an exception is a perfectly
        // good answer but its result is not Ok, and retireAt uses that
        // to decide whether anything is still owed. A gateway would then send
        // two responses for one request, and the second desynchronises the
        // client for every transaction after it.
        at(index).onPassthrough = nullptr;

        // The handler is application code: the record is found again by its
        // slot afterwards rather than trusted to be at the same position.
        ModbusPendingOp* record = &at(index);
        const uint16_t opId = record->opId;
        if (deliver) deliver(result, pdu, (uint16_t)pduLen);
        index = positionOf(record);
        if (index != NO_OP && record->opId == opId) retireAt(index, result);
        return true;
    }

    // Decoded against the request it answers. A Modbus response carries no
    // address and a bit read carries no bit count, so the request is the only
    // source for either - and it is what lets a bit read report the number of
    // bits the caller asked for rather than a multiple of eight.
    const ModbusRequest request = bound(at(index));
    ModbusResponse response;
    const ModbusDecode decoded = mbpdu::decodeResponse(pdu, pduLen, request, response);

    if (decoded != ModbusDecode::Ok) {
        if (definitive) {
            // The transaction identifier says this frame answers THIS request,
            // and it does not decode. There is no better answer coming, so
            // waiting for the timeout only delays telling the caller something
            // we already know - and a timeout would report "the device did not
            // answer" when it plainly did.
            retireAt(index, ModbusResult::ProtocolError);
            return true;
        }

        // Matched only by position, so this may be another master's frame on a
        // shared bus. Left outstanding, so the real reply can still arrive and
        // the timeout still applies if it does not.
        return false;
    }

    // An exception, first and always. Noticing it one step too late retires the
    // read as though it had succeeded.
    if (response.isException) {
        retireAt(index, mb_exceptionToResult(response.exceptionCode));
        return true;
    }

    if (roundTrip) roundTrip->addSample(mb_elapsed(at(index).sentAt, now));

    // Unpacked into the caller's memory BEFORE the operation is retired, since
    // retiring runs the handler and the handler is entitled to read the values
    // it was waiting for.
    const bool ok = unpack(at(index), response);
    retireAt(index, ok ? ModbusResult::Ok : ModbusResult::ProtocolError);
    return true;
}

bool ModbusOperations::unpack(const ModbusPendingOp& op, const ModbusResponse& response) {
    if (op.resultKind == ModbusResultKind::None || !op.resultPtr) return true;
    if (!response.data && response.dataLen != 0) return false;

    switch (op.resultKind) {
        case ModbusResultKind::Bits: {
            // Bounded by what the CALLER asked for, never by what the response
            // declares. The two agree here because decodeResponse checked them
            // against each other, but the bound still comes from our side: a
            // peer's byte count is data, never a bound.
            const uint16_t wanted = op.request.quantity;
            if ((uint32_t)mb_packedByteCount(wanted) > response.dataLen) return false;
            for (uint16_t i = 0; i < wanted; i++) {
                *mb_elemAt<bool>(op.resultPtr, i, op.resultStride) =
                    mb_getPackedBit(response.data, i);
            }
            return true;
        }

        case ModbusResultKind::Registers: {
            const uint16_t wanted = op.request.quantity;
            if ((uint32_t)wanted * 2 > response.dataLen) return false;
            for (uint16_t i = 0; i < wanted; i++) {
                *mb_elemAt<uint16_t>(op.resultPtr, i, op.resultStride) =
                    mb_unpackBE16(response.data + (size_t)i * 2);
            }
            return true;
        }

        case ModbusResultKind::Text: {
            // Bounded in BYTES, not in registers.
            //
            // A register read writes exactly two bytes per register, but the
            // byte count a text response declares need not be even, so a bound
            // expressed in registers is half a byte short of what actually gets
            // copied. Bounding in registers and copying in bytes is exactly how
            // a peer overruns a caller's buffer.
            if (op.resultCapacity == 0) return false;
            char* out = (char*)op.resultPtr;
            const uint16_t room = (uint16_t)(op.resultCapacity - 1);
            const uint16_t take = response.dataLen < room ? response.dataLen : room;
            memcpy(out, response.data, take);
            out[take] = '\0';
            return true;
        }

        case ModbusResultKind::Raw: {
            if (op.resultCapacity == 0) return false;
            const uint16_t take =
                response.dataLen < op.resultCapacity ? response.dataLen : op.resultCapacity;
            memcpy(op.resultPtr, response.data, take);
            return true;
        }

        case ModbusResultKind::Typed: {
            const uint16_t perValue = mb_wideRegisters(op.wideType);
            if (perValue == 0) return false;

            const uint32_t needed = (uint32_t)op.wideCount * perValue * 2;
            if (needed > response.dataLen) return false;

            for (uint16_t i = 0; i < op.wideCount; i++) {
                const uint8_t* wire = response.data + (size_t)i * perValue * 2;
                switch (op.wideType) {
                    case ModbusWideType::Float32:
                        *mb_elemAt<float>(op.resultPtr, i, op.resultStride) =
                            mb_toFloat32(wire, op.wideOrder);
                        break;
                    case ModbusWideType::Int32:
                        *mb_elemAt<int32_t>(op.resultPtr, i, op.resultStride) =
                            mb_toInt32(wire, op.wideOrder);
                        break;
                    case ModbusWideType::UInt32:
                        *mb_elemAt<uint32_t>(op.resultPtr, i, op.resultStride) =
                            mb_toUInt32(wire, op.wideOrder);
                        break;
                    case ModbusWideType::Float64:
                        *mb_elemAt<double>(op.resultPtr, i, op.resultStride) =
                            mb_toFloat64(wire, op.wideOrder);
                        break;
                    case ModbusWideType::Int64:
                        *mb_elemAt<int64_t>(op.resultPtr, i, op.resultStride) =
                            mb_toInt64(wire, op.wideOrder);
                        break;
                    case ModbusWideType::UInt64:
                        *mb_elemAt<uint64_t>(op.resultPtr, i, op.resultStride) =
                            mb_toUInt64(wire, op.wideOrder);
                        break;
                    case ModbusWideType::None:
                        return false;
                }
            }
            return true;
        }

        case ModbusResultKind::Probe: {
            if (response.dataLen < 4) return false;
            const uint16_t pair[2] = { mb_unpackBE16(response.data),
                                       mb_unpackBE16(response.data + 2) };
            *(ModbusOrderProbe*)op.resultPtr = mb_probeOrder(pair);
            return true;
        }
        case ModbusResultKind::Fifo: {
            // Registers on the wire are big-endian, so they are unpacked one by
            // one like any other register read - a byte copy would hand a
            // little-endian caller every value byte-swapped. As many as were
            // queued, up to the room the caller gave.
            const uint16_t take =
                response.quantity < op.resultCapacity ? response.quantity : op.resultCapacity;
            if ((uint32_t)take * 2 > response.dataLen) return false;
            for (uint16_t i = 0; i < take; i++) {
                *mb_elemAt<uint16_t>(op.resultPtr, i, 0) =
                    mb_unpackBE16(response.data + (size_t)i * 2);
            }
            return true;
        }

        case ModbusResultKind::None:
            return true;
    }
    return true;
}

// =============================================================================
// EXPIRY
// =============================================================================

void ModbusOperations::expire(mb_time_t now, mb_time_t timeoutMs, uint8_t maxRetries,
                              uint32_t bitsPerSecond, int16_t onlyUnit,
                              const uint32_t* skipUnits) {
    // Read through mb_retryLimit by the caller: a budget above 255 compared
    // against a byte counter never comes true, so a lost operation would never
    // report a timeout.
    const uint8_t limit = maxRetries;

    // Walked by INDEX rather than by iterator: retireAt() calls application
    // code, which may queue an operation and reallocate the vector out from
    // under an iterator.
    for (size_t i = 0; i < _count;) {
        ModbusPendingOp& op = at(i);

        if (onlyUnit >= 0 && op.unit != (uint8_t)onlyUnit) { i++; continue; }
        if (skipUnits != nullptr &&
            (skipUnits[op.unit >> 5] & (1u << (op.unit & 31))) != 0) {
            i++;
            continue;
        }

        if (!op.awaitingResponse) {
            // Queued, and never transmitted.
            //
            // The medium has refused it every time it has been offered - the
            // link is down, a socket has gone, a port will not take bytes - and
            // without a bound here that operation NEVER CONCLUDES. Not a
            // timeout, not a failure, nothing: the callback is never called and
            // status() answers Pending for ever.
            //
            // That is the one-conclusion contract broken in exactly the case an
            // application most needs to be told about, and it does not look
            // like a bug. It looks like a control loop that stopped asking, on
            // a plant where the only thing that changed was a cable.
            //
            // The budget is deliberately generous - several times a response
            // timeout - because an operation waiting behind a slow bus, or
            // through a reconnection, is perfectly healthy and must not be
            // concluded early.
            // Computed in 64 bits and clamped, because a caller may pin a
            // very large timeout and a 32-bit product would wrap to something
            // small - turning a generous budget into an instant conclusion,
            // which is the opposite of the intent.
            const uint64_t wide = (uint64_t)timeoutMs * (uint64_t)(limit + 1) * 4ull + 1000ull;
            const mb_time_t unsentBudget =
                wide > 0x7FFFFFFFull ? (mb_time_t)0x7FFFFFFFu : (mb_time_t)wide;
            if (mb_elapsed(op.queuedAt, now) > unsentBudget) {
                _neverSent++;
                retireAt(i, ModbusResult::LinkDown);
                continue;   // The entry at i is now whatever followed it
            }
            i++;
            continue;
        }

        // The device's thinking time, plus what the medium needs to carry this
        // particular exchange. On a 9600 line a 125-register read spends 300 ms
        // simply being clocked out, and a timeout that ignores that declares a
        // healthy device dead every time it is asked a large question.
        mb_time_t allowance = timeoutMs;
        if (bitsPerSecond) {
            const size_t requestBytes = 1 + op.payloadLen + 6;   // addr + PDU + CRC
            const size_t responseBytes = 3 + mb_expectedResponseBytes(op.request);
            allowance += (mb_time_t)(mb_airtimeUs(bitsPerSecond,
                                                  requestBytes + responseBytes) / 1000);
        }

        if (mb_elapsed(op.queuedAt, now) <= allowance) { i++; continue; }

        if (op.retryCount < limit) {
            op.retryCount++;
            op.queuedAt = now;
            op.awaitingResponse = false;   // Allow it to be sent again
            op.opId = nextOpId();          // A new transaction, so a late reply
                                           // to the old one cannot be matched
            i++;
            continue;
        }

        // Out of retries. "Nothing came back" is an answer the application
        // needs: dropped silently, a lost setpoint looks exactly like a
        // delivered one.
        retireAt(i, ModbusResult::Timeout);
        // The entry at i is now whatever followed it, so i does not advance.
    }
}

void ModbusOperations::cancelAll(ModbusResult why) {
    // Bounded by what is here NOW, not by "until empty".
    //
    // A completion handler is application code, and the most natural thing for
    // one to do is start the next request - which is exactly what shutdown must
    // not be defeated by. Draining until empty then never terminates: every
    // cancellation queues another operation to cancel, and end() hangs the
    // process at full CPU with no diagnostic and nothing an application can do
    // about it.
    //
    // Cancelling a snapshot instead means anything a handler queued during the
    // cancellation survives. That is the right answer: it is a NEW operation,
    // not one this call was asked to conclude, and an application shutting down
    // calls end(), which closes the link underneath it.
    size_t remaining = _count;
    while (remaining > 0 && !(_count == 0)) {
        const size_t index = _count < remaining ? _count - 1 : remaining - 1;
        retireAt(index, why);
        remaining--;
    }
}

void ModbusOperations::cancelUnit(uint8_t unit, ModbusResult why) {
    for (size_t i = 0; i < _count;) {
        if (at(i).unit == unit) {
            retireAt(i, why);
        } else {
            i++;
        }
    }
}

// =============================================================================
// CONCLUSION
// =============================================================================

void ModbusOperations::retireAt(size_t index, ModbusResult result) {
    if (index >= _count) return;

    ModbusPendingOp* record = &at(index);

    // A passthrough that never got a response is still owed its one report -
    // with no bytes, because there are none. Without this, a gateway waiting
    // on a downstream device that has gone would hold the client's socket open
    // for ever rather than answering 0x0B.
    if (record->passthrough && record->onPassthrough && result != ModbusResult::Ok) {
        const ModbusPassthroughHandler deliver = std::move(record->onPassthrough);
        record->onPassthrough = nullptr;
        const uint16_t opId = record->opId;
        deliver(result, nullptr, 0);
        // Application code ran: found again by its slot, and left alone if it
        // was concluded meanwhile.
        index = positionOf(record);
        if (index == NO_OP || record->opId != opId) return;
    }

    // Everything needed from the record is taken BEFORE it is removed, and
    // nothing reads it afterwards. This is the rule the whole class follows.
    ModbusCompletionHandler handler = std::move(record->onComplete);
    const uint8_t unit = record->unit;
    const uint16_t address = record->request.address;

    removeAt(index);

    noteFinished(unit, address, result);
    dispatch(Owed{ std::move(handler), result });
}

void ModbusOperations::drainCarriedDeferrals() {
    if (_deferred.count == 0 || _dispatchDepth > 0) return;

    // Only those carried over: running one may conclude more work, which is
    // queued behind them and goes round again on the next poll.
    const size_t carried = _deferred.count;
    _dispatchDepth++;
    for (size_t i = 0; i < carried && i < kMaxDeferredPerDispatch; i++) {
        const Owed owed = _deferred.pop();
        owed.done(owed.result);
    }
    if (carried > kMaxDeferredPerDispatch) _deferredCarried++;
    _dispatchDepth--;
}

void ModbusOperations::dispatch(Owed owed) {
    if (!owed.done) return;

    if (_dispatchDepth > 0) {
        // A conclusion reached while a handler is already running is queued
        // rather than nested, and drained by the outermost call below. Every
        // handler then runs at the same stack depth, whatever it does - unless
        // the ring is full, when it runs now rather than never.
        if (!reserve() || !_deferred.push(std::move(owed))) owed.done(owed.result);
        return;
    }

    _dispatchDepth++;
    owed.done(owed.result);

    // Drained with an index rather than an iterator: a deferred handler may
    // defer another.
    //
    // And bounded, because "may defer another" has no natural end. A handler
    // that concludes an operation from inside a handler - a local read, say,
    // which completes the moment it is queued - can feed this loop for as long
    // as it keeps doing it. Every entry is a real conclusion owed to somebody,
    // so none is discarded; the drain simply stops after a sane depth and the
    // rest are carried to the next poll(), which keeps one scan bounded without
    // breaking the one-conclusion-per-operation contract.
    size_t drained = 0;
    while (_deferred.count && drained < kMaxDeferredPerDispatch) {
        // Taken out of the ring before it runs: it may defer another.
        const Owed next = _deferred.pop();
        drained++;
        next.done(next.result);
    }

    // Anything left waits for the next poll() rather than being dropped.
    if (_deferred.count) _deferredCarried++;
    _dispatchDepth--;
}

// =============================================================================
// POLLED STATE
// =============================================================================

void ModbusOperations::noteFinished(uint8_t unit, uint16_t address, ModbusResult result) {
    if (result == ModbusResult::Queued) return;   // Not a conclusion

    for (size_t i = 0; i < _finishedCount; i++) {
        if (_finished[i].unit == unit && _finished[i].address == address) {
            _finished[i].result = result;
            return;
        }
    }
    // Bounded, and that bound is not a detail.
    //
    // This table exists for status(), which lets a caller ask how a request
    // ended rather than be told. It is keyed by (unit, address), so a client
    // reading a FIXED set of points settles at that many entries - but one
    // reading varying addresses (an HMI browsing registers, a gateway
    // forwarding whatever a client asks for, a scan) would grow it without
    // limit. On a microcontroller that is the heap gone in an afternoon, with
    // the symptom appearing days after commissioning.
    //
    // The oldest entry goes when the table is full. A caller polling status()
    // reads it long before then; one that never polls has no use for it at all.
    if (_finishedCount >= kMaxFinished) {
        memmove(_finished, _finished + 1, (kMaxFinished - 1) * sizeof _finished[0]);
        _finishedCount--;
        _finishedDropped++;
    }

    Finished& f = _finished[_finishedCount++];
    f.unit = unit;
    f.address = address;
    f.result = result;
}

void ModbusOperations::dropFinished(uint8_t unit, uint16_t address) {
    for (size_t i = 0; i < _finishedCount; i++) {
        if (_finished[i].unit == unit && _finished[i].address == address) {
            memmove(_finished + i, _finished + i + 1,
                    (_finishedCount - i - 1) * sizeof _finished[0]);
            _finishedCount--;
            return;
        }
    }
}

ModbusRequestState ModbusOperations::state(uint8_t unit, uint16_t address,
                                           ModbusResult& result) const {
    for (size_t i = 0; i < _count; i++) {
        if (at(i).unit == unit && at(i).request.address == address) {
            result = ModbusResult::Queued;
            return ModbusRequestState::Pending;
        }
    }
    for (size_t i = 0; i < _finishedCount; i++) {
        if (_finished[i].unit == unit && _finished[i].address == address) {
            result = _finished[i].result;
            return mb_succeeded(result) ? ModbusRequestState::Done
                                        : ModbusRequestState::Failed;
        }
    }
    result = ModbusResult::Queued;
    return ModbusRequestState::Idle;
}
