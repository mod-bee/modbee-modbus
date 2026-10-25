#include "ModbusPoint.h"

#include <math.h>
#include <new>
#include "ModbusNoExcept.h"

const char* mb_pointTypeName(ModbusPointType type) {
    switch (type) {
        case ModbusPointType::Bit:         return "BIT";
        case ModbusPointType::Int16:       return "INT16";
        case ModbusPointType::UInt16:      return "UINT16";
        case ModbusPointType::Int32:       return "INT32";
        case ModbusPointType::UInt32:      return "UINT32";
        case ModbusPointType::Float32:     return "FLOAT32";
        case ModbusPointType::RegisterBit: return "REGISTER_BIT";
    }
    return "?";
}

uint8_t mb_pointRegisters(ModbusPointType type) {
    switch (type) {
        case ModbusPointType::Bit:         return 0;
        case ModbusPointType::Int16:
        case ModbusPointType::UInt16:
        case ModbusPointType::RegisterBit: return 1;
        case ModbusPointType::Int32:
        case ModbusPointType::UInt32:
        case ModbusPointType::Float32:     return 2;
    }
    return 0;
}

bool mb_pointValid(ModbusPointFc fc, ModbusPointType type, uint8_t bit) {
    if ((uint8_t)fc > (uint8_t)ModbusPointFc::WriteRegisters) return false;
    if ((uint8_t)type > (uint8_t)ModbusPointType::RegisterBit) return false;
    if (mb_pointFcBits(fc)) return type == ModbusPointType::Bit;
    if (type == ModbusPointType::Bit) return false;              // a register FC needs a register type
    if (type == ModbusPointType::RegisterBit) {
        // A bit of a register is read, never written: writing it would be a
        // read-modify-write that races every other writer of that register.
        return !mb_pointFcWrites(fc) && bit <= 15;
    }
    if (fc == ModbusPointFc::WriteRegister) return mb_pointRegisters(type) == 1;
    return true;
}

// One register in the requested order. ABCD and CDAB keep the bytes; BADC and
// DCBA are byte-swapped, which for a single register is all their names mean.
static uint16_t mb_pointWord(uint16_t word, ModbusOrder order) {
    if (order == ModbusOrder::BADC || order == ModbusOrder::DCBA) {
        return (uint16_t)((word >> 8) | (word << 8));
    }
    return word;
}

bool mb_pointDecode(const uint16_t* registers, uint8_t count, ModbusPointType type,
                    ModbusOrder order, uint8_t bit, double& raw) {
    if (count < mb_pointRegisters(type)) return false;
    switch (type) {
        case ModbusPointType::Bit:
            return false;   // bits do not come as registers
        case ModbusPointType::Int16:
            raw = (double)(int16_t)mb_pointWord(registers[0], order);
            return true;
        case ModbusPointType::UInt16:
            raw = (double)mb_pointWord(registers[0], order);
            return true;
        case ModbusPointType::RegisterBit:
            if (bit > 15) return false;
            raw = ((mb_pointWord(registers[0], order) >> bit) & 1u) ? 1.0 : 0.0;
            return true;
        case ModbusPointType::Int32:
            raw = (double)mb_registersToInt32(registers, order);
            return true;
        case ModbusPointType::UInt32:
            raw = (double)mb_registersToUInt32(registers, order);
            return true;
        case ModbusPointType::Float32:
            raw = (double)mb_registersToFloat32(registers, order);
            return true;
    }
    return false;
}

static double mb_pointClamp(double value, double low, double high) {
    if (value != value) return 0.0;   // NaN: nothing sensible to send
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

uint8_t mb_pointEncode(double raw, ModbusPointType type, ModbusOrder order,
                       uint16_t* registers) {
    switch (type) {
        case ModbusPointType::Int16: {
            const double v = mb_pointClamp(floor(raw + 0.5), -32768.0, 32767.0);
            registers[0] = mb_pointWord((uint16_t)(int16_t)v, order);
            return 1;
        }
        case ModbusPointType::UInt16: {
            const double v = mb_pointClamp(floor(raw + 0.5), 0.0, 65535.0);
            registers[0] = mb_pointWord((uint16_t)v, order);
            return 1;
        }
        case ModbusPointType::Int32: {
            const double v = mb_pointClamp(floor(raw + 0.5), -2147483648.0, 2147483647.0);
            mb_int32ToRegisters((int32_t)v, registers, order);
            return 2;
        }
        case ModbusPointType::UInt32: {
            const double v = mb_pointClamp(floor(raw + 0.5), 0.0, 4294967295.0);
            mb_uint32ToRegisters((uint32_t)v, registers, order);
            return 2;
        }
        case ModbusPointType::Float32:
            mb_float32ToRegisters((float)raw, registers, order);
            return 2;
        case ModbusPointType::Bit:
        case ModbusPointType::RegisterBit:
            return 0;
    }
    return 0;
}

// =============================================================================
// ONE POINT
// =============================================================================

bool ModbusPoint::configure(const ModbusPointConfig& config) {
    ModbusPointConfig wanted = config;
    if (wanted.everyMs == 0) wanted.everyMs = 1000;
    if (wanted.scale == 0.0f) wanted.scale = 1.0f;

    const bool retarget = !_configured || !wanted.sameTarget(_config);
    const bool resumed = wanted.enabled && (!_configured || !_config.enabled);
    _config = wanted;
    _configured = true;
    _valid = mb_pointValid(wanted.fc, wanted.type, wanted.bit);

    if (retarget) {
        // A DIFFERENT POINT. Nothing known about the old one describes this
        // one: not its value, not its age, not its failures. A reply still on
        // its way for the old address is discarded when it lands.
        _generation++;
        _inFlight = false;
        _attempted = false;
        _everGood = false;
        _failures = 0;
        _lastResult = _valid ? ModbusResult::Queued : ModbusResult::InvalidArgument;
        _value = 0.0;
        _hasWritten = false;
    } else if (resumed) {
        // Started again: read now rather than at the end of another interval.
        _attempted = false;
    }
    if (!_valid) _lastResult = ModbusResult::InvalidArgument;
    return _valid;
}

void ModbusPoint::setWriteValue(double value) { _writeValue = value; }

mb_time_t ModbusPoint::interval() const {
    return _config.everyMs != 0 ? _config.everyMs : 1000;
}

mb_time_t ModbusPoint::effectiveInterval() const {
    // Backed off while it fails - an absent device costs the whole bus a
    // timeout per attempt - but never beyond 8 intervals or a minute,
    // whichever is shorter (and never below one interval), so a radio device
    // that comes back is seen soon.
    const mb_time_t base = interval();
    if (_failures == 0) return base;
    uint32_t multiple = 1;
    for (uint16_t i = 0; i < _failures && multiple < 8; i++) multiple *= 2;
    uint64_t wanted = (uint64_t)base * multiple;
    const uint64_t cap = base > 60000u ? (uint64_t)base : 60000u;
    if (wanted > cap) wanted = cap;
    return (mb_time_t)wanted;
}

void ModbusPoint::service(ModbusClient& client, mb_time_t now) {
    if (!_configured || !_valid || !_config.enabled || _inFlight) return;

    const bool writes = mb_pointFcWrites(_config.fc);
    bool due = !_attempted || mb_elapsed(_lastAttemptMs, now) >= effectiveInterval();
    if (writes && !due && (!_hasWritten || _writeValue != _written) && _failures == 0) {
        // A new value goes now, not at the next refresh.
        due = true;
    }
    if (!due) return;

    _attempted = true;
    _lastAttemptMs = now;
    _inFlight = true;

    const ModbusCompletionHandler done(&ModbusPoint::completed, this, (uintptr_t)_generation);

    const uint8_t unit = _config.unit;
    const uint16_t address = _config.address;
    ModbusResult queued = ModbusResult::InvalidArgument;

    switch (_config.fc) {
        case ModbusPointFc::ReadCoils:
            queued = client.readCoils(unit, address, &_inboxBit, 1, done);
            break;
        case ModbusPointFc::ReadDiscreteInputs:
            queued = client.readDiscreteInputs(unit, address, &_inboxBit, 1, done);
            break;
        case ModbusPointFc::ReadHolding:
            queued = client.readHoldingRegisters(unit, address, _inbox,
                                                 mb_pointRegisters(_config.type), done);
            break;
        case ModbusPointFc::ReadInput:
            queued = client.readInputRegisters(unit, address, _inbox,
                                               mb_pointRegisters(_config.type), done);
            break;
        case ModbusPointFc::WriteCoil:
            _sending = _writeValue;
            queued = client.writeCoil(unit, address, _writeValue != 0.0, done);
            break;
        case ModbusPointFc::WriteRegister:
        case ModbusPointFc::WriteRegisters: {
            _sending = _writeValue;
            uint16_t regs[2] = {0, 0};
            const double raw = (_writeValue - (double)_config.offset) / (double)_config.scale;
            const uint8_t n = mb_pointEncode(raw, _config.type, _config.order, regs);
            if (n == 0) {
                // configure() refuses every type that cannot be written, so
                // this is a guard rather than a path: nothing was queued and
                // nothing will call back.
                complete(_generation, ModbusResult::InvalidArgument);
                return;
            }
            queued = (_config.fc == ModbusPointFc::WriteRegister)
                         ? client.writeRegister(unit, address, regs[0], done)
                         : client.writeRegisters(unit, address, regs, n, done);
            break;
        }
    }

    // Every result, a refusal included, concludes through `done` from the
    // client's poll() - the client reports refusals on its local path - so
    // there is nothing to conclude here.
    (void)queued;
}

void ModbusPoint::complete(uint32_t generation, ModbusResult result) {
    if (generation != _generation) return;   // an answer for what this point used to be
    _inFlight = false;

    if (result == ModbusResult::Duplicate) {
        // The same request from another point is already on its way: not a
        // failure of this device, so ask again next interval.
        return;
    }
    _lastResult = result;

    if (result != ModbusResult::Ok) {
        if (_failures < 0xFFFF) _failures++;
        return;
    }

    _failures = 0;
    _lastGoodMs = mb_millis();
    _everGood = true;
    _answers++;

    if (mb_pointFcWrites(_config.fc)) {
        _written = _sending;
        _hasWritten = true;
        _value = _sending;
        return;
    }

    double raw = 0.0;
    if (mb_pointFcBits(_config.fc)) {
        raw = _inboxBit ? 1.0 : 0.0;
        _value = raw;   // a bit is a bit: no scale or offset
        return;
    }
    if (mb_pointDecode(_inbox, 2, _config.type, _config.order, _config.bit, raw)) {
        _value = (_config.type == ModbusPointType::RegisterBit)
                     ? raw
                     : raw * (double)_config.scale + (double)_config.offset;
    }
}

ModbusQuality ModbusPoint::quality(mb_time_t now) const {
    if (!_everGood) return ModbusQuality::Unknown;
    if (_failures >= 2) return ModbusQuality::Bad;

    const mb_time_t every = interval();
    const uint64_t staleWide = _config.staleMs != 0 ? (uint64_t)_config.staleMs
                                                    : (uint64_t)every * 3u;
    const mb_time_t elapsed = mb_elapsed(_lastGoodMs, now);
    if (elapsed > staleWide) return ModbusQuality::Bad;
    if (_failures == 1) return ModbusQuality::Uncertain;
    // One interval's grace plus the round trip: an answer due now is not late.
    if ((uint64_t)elapsed <= (uint64_t)every * 2u) return ModbusQuality::Good;
    return ModbusQuality::Uncertain;
}

bool ModbusPoint::ok(mb_time_t now) const {
    const ModbusQuality q = quality(now);
    return q == ModbusQuality::Good || q == ModbusQuality::Uncertain;
}

mb_time_t ModbusPoint::age(mb_time_t now) const {
    if (!_everGood) return ModbusPoller::NEVER;
    return mb_elapsed(_lastGoodMs, now);
}

// =============================================================================
// THE SET
// =============================================================================

ModbusPointSet::~ModbusPointSet() {
    for (size_t i = 0; i < _points.size(); i++) delete _points[i];
}

size_t ModbusPointSet::add(size_t count) {
    const size_t first = _points.size();
    bool ok = _points.reserve(first + count);
    for (size_t i = 0; ok && i < count; i++) {
        ModbusPoint* point = new (std::nothrow) ModbusPoint();
        ok = point != nullptr && _points.push(point);
        if (!ok) delete point;
    }
    if (!ok) {
        for (size_t i = first; i < _points.size(); i++) delete _points[i];
        _points.erase(first, _points.size() - first);
        return (size_t)-1;
    }
    return first;
}

ModbusPoint* ModbusPointSet::at(size_t index) {
    return index < _points.size() ? _points[index] : nullptr;
}

void ModbusPointSet::poll(ModbusClient& client) {
    if (_points.empty()) return;

    if (_timeoutsDirty) {
        _timeoutsDirty = false;
        // Each unit takes the longest timeout any of its points asks for; a
        // unit that no longer has one goes back to measuring.
        uint32_t named[8] = { 0 };
        for (size_t i = 0; i < _points.size(); i++) {
            const ModbusPointConfig& c = _points[i]->config();
            if (c.timeoutMs == 0) continue;
            if (named[c.unit >> 5] & (1u << (c.unit & 31))) continue;
            mb_time_t longest = 0;
            for (size_t j = 0; j < _points.size(); j++) {
                const ModbusPointConfig& o = _points[j]->config();
                if (o.unit == c.unit && o.timeoutMs > longest) longest = o.timeoutMs;
            }
            client.setUnitTimeout(c.unit, longest);
            named[c.unit >> 5] |= 1u << (c.unit & 31);
        }
        for (unsigned unit = 0; unit < 256; unit++) {
            const uint32_t bit = 1u << (unit & 31);
            if ((_timedUnits[unit >> 5] & bit) && !(named[unit >> 5] & bit)) {
                client.setUnitTimeout((uint8_t)unit, 0);
            }
        }
        memcpy(_timedUnits, named, sizeof named);
    }

    // Every point gets its turn each poll, starting where the last left off:
    // the client's queue orders the bus, and a dead device at the top of the
    // table cannot starve the rest.
    const mb_time_t now = mb_millis();
    const size_t n = _points.size();
    for (size_t k = 0; k < n; k++) {
        _points[(_cursor + k) % n]->service(client, now);
    }
    _cursor = (_cursor + 1) % n;
}
