#include "ModbusPoller.h"

#include <string.h>
#include "ModbusNoExcept.h"

const char* mb_qualityName(ModbusQuality quality) {
    switch (quality) {
        case ModbusQuality::Unknown:   return "Unknown";
        case ModbusQuality::Good:      return "Good";
        case ModbusQuality::Uncertain: return "Uncertain";
        case ModbusQuality::Bad:       return "Bad";
    }
    return "Unknown";
}

// =============================================================================
// DECLARING
// =============================================================================

size_t ModbusPoller::add(const Poll& poll) {
    if (!poll.dest || poll.count == 0 || poll.intervalMs == 0) return (size_t)-1;
    _polls.push(poll);
    return _polls.size() - 1;
}

size_t ModbusPoller::addRegisters(uint8_t unit, uint16_t address, uint16_t* dest,
                                  uint16_t count, mb_time_t intervalMs,
                                  ModbusTable table, uint16_t stride) {
    if (mb_tableIsBits(table)) return (size_t)-1;
    Poll poll;
    poll.unit = unit;
    poll.address = address;
    poll.count = count;
    poll.table = table;
    poll.dest = dest;
    poll.stride = stride;
    poll.intervalMs = intervalMs;
    return add(poll);
}

size_t ModbusPoller::addBits(uint8_t unit, uint16_t address, bool* dest, uint16_t count,
                             mb_time_t intervalMs, ModbusTable table,
                             uint16_t stride) {
    if (!mb_tableIsBits(table)) return (size_t)-1;
    Poll poll;
    poll.unit = unit;
    poll.address = address;
    poll.count = count;
    poll.table = table;
    poll.dest = dest;
    poll.stride = stride;
    poll.bits = true;
    poll.intervalMs = intervalMs;
    return add(poll);
}

size_t ModbusPoller::addFloat32(uint8_t unit, uint16_t address, float* dest,
                                mb_time_t intervalMs, ModbusOrder order) {
    Poll poll;
    poll.unit = unit;
    poll.address = address;
    poll.count = 1;
    poll.dest = dest;
    poll.wideType = ModbusWideType::Float32;
    poll.order = order;
    poll.intervalMs = intervalMs;
    return add(poll);
}

size_t ModbusPoller::addInt32(uint8_t unit, uint16_t address, int32_t* dest,
                              mb_time_t intervalMs, ModbusOrder order) {
    Poll poll;
    poll.unit = unit;
    poll.address = address;
    poll.count = 1;
    poll.dest = dest;
    poll.wideType = ModbusWideType::Int32;
    poll.order = order;
    poll.intervalMs = intervalMs;
    return add(poll);
}

size_t ModbusPoller::addWide(uint8_t unit, uint16_t address, void* dest,
                             ModbusWideType type, uint16_t count, mb_time_t intervalMs,
                             ModbusOrder order, ModbusTable table, uint16_t stride) {
    // A bit list cannot carry a 32-bit value, and a wide type of None is not a
    // wide value at all.
    if (type == ModbusWideType::None) return (size_t)-1;
    if (table != ModbusTable::HoldingRegister && table != ModbusTable::InputRegister) {
        return (size_t)-1;
    }
    Poll poll;
    poll.unit = unit;
    poll.address = address;
    poll.count = count;
    poll.table = table;
    poll.dest = dest;
    poll.stride = stride;
    poll.wideType = type;
    poll.order = order;
    poll.intervalMs = intervalMs;
    return add(poll);
}

void ModbusPoller::remove(size_t index) {
    // Deactivated rather than erased, so every other index a caller is holding
    // stays valid. An index the caller cannot rely on is not an index.
    if (index < _polls.size()) _polls[index].active = false;
}

void ModbusPoller::resume(size_t index) {
    if (index >= _polls.size()) return;
    _polls[index].active = true;

    // Due at once rather than at the end of another interval. A watch that has
    // just been started should read now; making the caller wait a second for a
    // value they asked for this instant is the wrong answer.
    _polls[index].lastAttemptMs = 0;
}

void ModbusPoller::setInterval(size_t index, mb_time_t intervalMs) {
    if (index >= _polls.size() || intervalMs == 0) return;
    _polls[index].intervalMs = intervalMs;
}

bool ModbusPoller::retarget(size_t index, uint8_t unit, uint16_t address,
                            ModbusTable table, ModbusOrder order) {
    if (index >= _polls.size()) return false;
    Poll& poll = _polls[index];
    if (mb_tableIsBits(table) != poll.bits) return false;
    if (poll.wideType != ModbusWideType::None &&
        table != ModbusTable::HoldingRegister && table != ModbusTable::InputRegister) {
        return false;
    }
    poll.unit = unit;
    poll.address = address;
    poll.table = table;
    poll.order = order;
    poll.everSucceeded = false;
    poll.failures = 0;
    poll.lastResult = ModbusResult::Queued;
    poll.hasPrevious = false;
    poll.lastAttemptMs = 0;          // read the new address now
    if (poll.inFlight) {
        if (poll.discard < 0xFF) poll.discard++;
        poll.inFlight = false;       // the new address need not wait for the old reply
    }
    return true;
}

bool ModbusPoller::isActive(size_t index) const {
    return index < _polls.size() && _polls[index].active;
}

void ModbusPoller::clear() {
    _polls.clear();
    _cursor = 0;
}

void ModbusPoller::setDeadband(size_t index, uint16_t deadband) {
    if (index < _polls.size()) _polls[index].deadband = deadband;
}

// =============================================================================
// TRUST
// =============================================================================

mb_time_t ModbusPoller::age(size_t index) const {
    if (index >= _polls.size()) return NEVER;
    const Poll& poll = _polls[index];
    if (!poll.everSucceeded) return NEVER;
    return mb_elapsed(poll.lastSuccessMs, mb_millis());
}

ModbusQuality ModbusPoller::quality(size_t index) const {
    if (index >= _polls.size()) return ModbusQuality::Unknown;
    const Poll& poll = _polls[index];

    // Never read at all. Deliberately NOT Bad: one is a commissioning problem
    // and the other is a running fault, and an alarm list that conflates them
    // sends somebody to the wrong end of the plant.
    if (!poll.everSucceeded) return ModbusQuality::Unknown;

    // The last attempt was refused. Whatever is in the variable is now known to
    // be out of date, however recent it is.
    if (poll.failures > 0 && poll.lastResult != ModbusResult::Ok) {
        return poll.failures >= 2 ? ModbusQuality::Bad : ModbusQuality::Uncertain;
    }

    // Judged against the interval THE CALLER ASKED FOR, not against a figure
    // this class invented. A value polled every ten minutes is not stale at
    // nine; one polled every hundred milliseconds is stale at one second.
    const mb_time_t elapsed = mb_elapsed(poll.lastSuccessMs, mb_millis());

    // One interval's grace, because a poll issued exactly on schedule still
    // takes a round trip to come back. Beyond two, something is wrong.
    if (elapsed <= poll.intervalMs * 2) return ModbusQuality::Good;
    if (elapsed <= poll.intervalMs * 4) return ModbusQuality::Uncertain;
    return ModbusQuality::Bad;
}

ModbusResult ModbusPoller::lastResult(size_t index) const {
    return index < _polls.size() ? _polls[index].lastResult : ModbusResult::Queued;
}

uint16_t ModbusPoller::consecutiveFailures(size_t index) const {
    return index < _polls.size() ? _polls[index].failures : 0;
}

bool ModbusPoller::allGood() const {
    for (size_t i = 0; i < _polls.size(); i++) {
        if (!_polls[i].active) continue;
        if (quality(i) != ModbusQuality::Good) return false;
    }
    return true;
}

// =============================================================================
// DRIVING
// =============================================================================

mb_time_t ModbusPoller::effectiveInterval(const Poll& poll) const {
    if (poll.failures == 0 || _maxBackoff <= 1) return poll.intervalMs;

    // Doubling per consecutive failure, capped. A device that has gone must not
    // keep asking at full rate: on a serial segment only one transaction is in
    // flight at a time, so every attempt on an absent unit costs the WHOLE BUS
    // a response timeout, and a handful of dead devices can consume a segment
    // entirely while the live ones go stale.
    //
    // It snaps back the instant the device answers, so a unit that comes back
    // is picked up on its next scheduled poll rather than after a penalty.
    uint32_t multiple = 1;
    for (uint16_t i = 0; i < poll.failures && multiple < _maxBackoff; i++) {
        multiple *= 2;
    }
    if (multiple > _maxBackoff) multiple = _maxBackoff;
    return poll.intervalMs * multiple;
}

bool ModbusPoller::due(const Poll& poll, mb_time_t now) const {
    if (!poll.active || poll.inFlight) return false;
    if (poll.lastAttemptMs == 0) return true;   // Never attempted
    return mb_elapsed(poll.lastAttemptMs, now) >= effectiveInterval(poll);
}

uint32_t ModbusPoller::snapshot(const Poll& poll) const {
    // The first element's value, as a number the deadband can be applied to. A
    // deadband over a whole block is a question with no single answer, so it is
    // deliberately not offered - see the note on setDeadband.
    if (poll.wideType == ModbusWideType::Float32) {
        uint32_t bits;
        const float value = *(const float*)poll.dest;
        memcpy(&bits, &value, sizeof bits);
        return bits;
    }
    if (poll.wideType == ModbusWideType::Int32) {
        return (uint32_t)*(const int32_t*)poll.dest;
    }
    if (poll.bits) return *(const bool*)poll.dest ? 1u : 0u;
    return *(const uint16_t*)poll.dest;
}

void ModbusPoller::issue(size_t index, mb_time_t now) {
    Poll& poll = _polls[index];
    poll.lastAttemptMs = now;
    poll.inFlight = true;

    const ModbusCompletionHandler done(&ModbusPoller::completed, this, (uintptr_t)index);

    ModbusResult queued;
    if (poll.wideType != ModbusWideType::None) {
        queued = _client.readWideArray(poll.unit, poll.address, poll.dest, poll.wideType,
                                       poll.order, poll.count, poll.stride, done,
                                       poll.table);
    } else if (poll.bits) {
        queued = poll.table == ModbusTable::Coil
                     ? _client.readCoils(poll.unit, poll.address, (bool*)poll.dest,
                                         poll.count, done, poll.stride)
                     : _client.readDiscreteInputs(poll.unit, poll.address,
                                                  (bool*)poll.dest, poll.count, done,
                                                  poll.stride);
    } else {
        queued = poll.table == ModbusTable::HoldingRegister
                     ? _client.readHoldingRegisters(poll.unit, poll.address,
                                                    (uint16_t*)poll.dest, poll.count, done,
                                                    poll.stride)
                     : _client.readInputRegisters(poll.unit, poll.address,
                                                  (uint16_t*)poll.dest, poll.count, done,
                                                  poll.stride);
    }

    // A refusal concludes through the same handler as a success, so there is
    // nothing to do here - except where the queue refused it outright and will
    // therefore never call back, which `add` reports as a non-Queued result and
    // the client already routes through its local completion path.
    (void)queued;
}

void ModbusPoller::complete(size_t index, ModbusResult result) {
    if (index >= _polls.size()) return;
    Poll& poll = _polls[index];
    if (poll.discard > 0) {
        // The reply to a read issued before retarget(). Whatever it carried
        // describes the old address; the next read is for the new one. The
        // client concludes in order, so the orphans arrive first.
        poll.discard--;
        return;
    }
    poll.inFlight = false;
    poll.lastResult = result;

    if (result != ModbusResult::Ok) {
        // Counted rather than latched: one failure is a bus hiccup, ten is a
        // device that has gone, and an alarm should be able to tell them apart.
        if (poll.failures < 0xFFFF) poll.failures++;
        if (_pollHandler) _pollHandler(index, result);
        return;
    }

    poll.failures = 0;
    poll.lastSuccessMs = mb_millis();
    poll.everSucceeded = true;

    const uint32_t now = snapshot(poll);
    bool changed = !poll.hasPrevious;

    if (poll.hasPrevious && poll.count == 1) {
        if (poll.wideType == ModbusWideType::Float32) {
            // Compared as floats, because a deadband on the bit pattern of a
            // float means nothing at all.
            float previous, current;
            memcpy(&previous, &poll.previous, sizeof previous);
            memcpy(&current, &now, sizeof current);
            const float delta = current > previous ? current - previous : previous - current;
            changed = delta > (float)poll.deadband;
        } else {
            const uint32_t delta = now > poll.previous ? now - poll.previous
                                                       : poll.previous - now;
            changed = delta > poll.deadband;
        }
    } else if (poll.hasPrevious) {
        // A block has no single value to apply a deadband to, so every
        // successful read of one counts as a change. Saying so is better than
        // inventing a rule nobody asked for.
        changed = true;
    }

    poll.previous = now;
    poll.hasPrevious = true;

    if (_pollHandler) _pollHandler(index, result);
    if (changed && _changeHandler) _changeHandler(index, poll.address);
}

void ModbusPoller::poll() {
    if (_polls.empty()) return;

    const mb_time_t now = mb_millis();

    // ONE poll per call, starting where the last scan left off.
    //
    // Two reasons, and both matter on a real bus. Queueing everything at once
    // fills the operation queue with work the bus cannot carry, so the last
    // entry's value is as old as the whole scan by the time it arrives. And a
    // rotating cursor means one slow or failing entry cannot starve the rest -
    // without it, an absent device at index 0 would hold the scan at its
    // timeout for ever.
    for (size_t attempt = 0; attempt < _polls.size(); attempt++) {
        const size_t index = (_cursor + attempt) % _polls.size();
        if (!due(_polls[index], now)) continue;

        issue(index, now);
        _cursor = (index + 1) % _polls.size();
        return;
    }
}

void ModbusPoller::refreshAll() {
    // Everything becomes due at once - after a reconnect, say, where holding to
    // the schedule would leave the first values as old as the outage.
    for (size_t i = 0; i < _polls.size(); i++) _polls[i].lastAttemptMs = 0;
}
