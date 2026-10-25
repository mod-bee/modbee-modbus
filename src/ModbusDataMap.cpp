#include "ModbusDataMap.h"

#include <string.h>
#include "ModbusNoExcept.h"

// =============================================================================
// BINDING LOOKUP
// =============================================================================

bool ModbusDataMap::Binding::writable() const {
    switch (kind) {
        case Kind::BitHandler:      return (bool)bitWriter;
        case Kind::RegisterHandler: return (bool)regWriter;
        default:                    return mb_tableIsWritable(table);
    }
}

namespace {

// Ordering for the sorted binding vector: table first, then start address.
// Bindings within one table never overlap, so start alone totally orders them.
bool bindingLess(const ModbusDataMap* /*unused*/, ModbusTable table, uint16_t start,
                 ModbusTable otherTable, uint16_t otherStart) {
    if (table != otherTable) return (uint8_t)table < (uint8_t)otherTable;
    return start < otherStart;
}

} // namespace

ModbusDataMap::ModbusDataMap() noexcept {}
ModbusDataMap::~ModbusDataMap() {}

const ModbusDataMap::Binding* ModbusDataMap::find(ModbusTable table, uint16_t address) const {
    // Binary search for the last binding at or before `address` in this table,
    // then check that the address really falls inside it. A block binding
    // covers addresses its start does not equal, so a plain equality lookup
    // would find only the first register of every block.
    size_t lo = 0, hi = _bindings.size();
    while (lo < hi) {
        const size_t mid = lo + (hi - lo) / 2;
        const Binding& b = _bindings[mid];
        if (bindingLess(this, b.table, b.start, table, address) ||
            (b.table == table && b.start == address)) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    if (lo == 0) return nullptr;

    const Binding& candidate = _bindings[lo - 1];
    if (candidate.table != table) return nullptr;
    if (address < candidate.start) return nullptr;
    if ((uint32_t)address >= (uint32_t)candidate.start + candidate.count) return nullptr;
    return &candidate;
}

ModbusDataMap::Binding* ModbusDataMap::find(ModbusTable table, uint16_t address) {
    const ModbusDataMap* self = this;
    return const_cast<Binding*>(self->find(table, address));
}

bool ModbusDataMap::overlaps(ModbusTable table, uint16_t address, uint16_t count) const {
    if (count == 0) return true;   // A binding of nothing is not a binding

    // Checked with 32-bit arithmetic: a range that runs past 65535 must be
    // rejected here rather than wrapping to address 0 and colliding with
    // whatever is bound there. The same rule as for a request, applied at bind
    // time so the collision cannot happen at all.
    const uint32_t end = (uint32_t)address + count;
    if (end > 0x10000u) return true;

    for (const Binding& b : _bindings) {
        if (b.table != table) continue;
        const uint32_t bEnd = (uint32_t)b.start + b.count;
        if (address < bEnd && b.start < end) return true;
    }
    return false;
}

bool ModbusDataMap::insert(const Binding& binding) {
    if (overlaps(binding.table, binding.start, binding.count)) return false;

    // Keep the list sorted, so find() can binary search it.
    size_t position = 0;
    while (position < _bindings.size()) {
        const Binding& b = _bindings[position];
        if (b.table != binding.table ? (uint8_t)b.table > (uint8_t)binding.table
                                     : b.start > binding.start) {
            break;
        }
        position++;
    }
    return _bindings.insert(position, binding);
}

// =============================================================================
// BINDING
// =============================================================================

bool ModbusDataMap::bind(ModbusTable table, uint16_t address, bool* variable) {
    return bindBlock(table, address, variable, 1, 0);
}

bool ModbusDataMap::bind(ModbusTable table, uint16_t address, uint16_t* variable) {
    return bindBlock(table, address, variable, 1, 0);
}

bool ModbusDataMap::bindBlock(ModbusTable table, uint16_t address, bool* array,
                              uint16_t count, uint16_t stride) {
    // A bit array belongs in a bit table. Binding one over holding registers
    // would mean a bool per register, which is not what anybody means by it.
    if (!array || count == 0 || !mb_tableIsBits(table)) return false;

    Binding b;
    b.table = table;
    b.kind = Kind::Bits;
    b.start = address;
    b.count = count;
    b.bits = array;
    b.stride = stride;
    return insert(b);
}

bool ModbusDataMap::bindBlock(ModbusTable table, uint16_t address, uint16_t* array,
                              uint16_t count, uint16_t stride) {
    if (!array || count == 0 || mb_tableIsBits(table)) return false;

    Binding b;
    b.table = table;
    b.kind = Kind::Registers;
    b.start = address;
    b.count = count;
    b.registers = array;
    b.stride = stride;
    return insert(b);
}

bool ModbusDataMap::bindString(ModbusTable table, uint16_t address, char* buffer,
                               uint16_t capacity) {
    // Two characters to a register. An odd capacity leaves its final byte
    // unused rather than half-occupying a register nothing can address.
    if (!buffer || capacity < 2 || mb_tableIsBits(table)) return false;

    Binding b;
    b.table = table;
    b.kind = Kind::String;
    b.start = address;
    b.count = (uint16_t)(capacity / 2);
    b.text = buffer;
    b.textCapacity = capacity;
    if (b.count == 0) return false;
    if (!insert(b)) return false;

    // The last byte is the terminator, always, so a buffer that arrives with
    // rubbish in it does not read back as an unterminated string.
    buffer[capacity - 1] = '\0';
    return true;
}

bool ModbusDataMap::bindHandler(ModbusTable table, uint16_t address, uint16_t count,
                                ModbusRegisterReader reader, ModbusRegisterWriter writer) {
    if (!reader || count == 0 || mb_tableIsBits(table)) return false;

    Binding b;
    b.table = table;
    b.kind = Kind::RegisterHandler;
    b.start = address;
    b.count = count;
    b.regReader = reader;
    b.regWriter = writer;
    return insert(b);
}

bool ModbusDataMap::bindHandler(ModbusTable table, uint16_t address, uint16_t count,
                                ModbusBitReader reader, ModbusBitWriter writer) {
    if (!reader || count == 0 || !mb_tableIsBits(table)) return false;

    Binding b;
    b.table = table;
    b.kind = Kind::BitHandler;
    b.start = address;
    b.count = count;
    b.bitReader = reader;
    b.bitWriter = writer;
    return insert(b);
}

bool ModbusDataMap::unbind(ModbusTable table, uint16_t address) {
    for (size_t i = 0; i < _bindings.size(); i++) {
        if (_bindings[i].table == table && _bindings[i].start == address) {
            _bindings.erase(i);
            return true;
        }
    }
    return false;
}

void ModbusDataMap::clear() {
    _bindings.clear();
    _safeStates.clear();
    _watches.clear();
    _pendingChanges.clear();
    _writeCounts.clear();
    _failsafeHandler = nullptr;
}

// =============================================================================
// EXISTENCE
// =============================================================================

bool ModbusDataMap::has(ModbusTable table, uint16_t address) const {
    return find(table, address) != nullptr;
}

bool ModbusDataMap::hasRange(ModbusTable table, uint16_t address, uint16_t count) const {
    if (count == 0) return false;
    if (!mb_isValidAddressRange(address, count)) return false;

    // Walked binding by binding rather than address by address: a block of a
    // thousand registers is one lookup, not a thousand.
    uint32_t at = address;
    const uint32_t end = (uint32_t)address + count;
    while (at < end) {
        const Binding* b = find(table, (uint16_t)at);
        if (!b) return false;
        at = (uint32_t)b->start + b->count;
    }
    return true;
}

bool ModbusDataMap::hasWritableRange(ModbusTable table, uint16_t address,
                                     uint16_t count) const {
    if (!hasRange(table, address, count)) return false;

    uint32_t at = address;
    const uint32_t end = (uint32_t)address + count;
    while (at < end) {
        const Binding* b = find(table, (uint16_t)at);
        if (!b || !b->writable()) return false;
        at = (uint32_t)b->start + b->count;
    }
    return true;
}

// =============================================================================
// SPAN HELPERS
// =============================================================================

uint16_t ModbusDataMap::spanRegister(const Binding& span, uint16_t offset) {
    // Two characters to a register, first character in the high byte. Reading
    // past the terminator gives zeroes rather than whatever the buffer holds.
    const uint16_t hiIndex = (uint16_t)(offset * 2);
    const uint16_t loIndex = (uint16_t)(hiIndex + 1);
    const uint8_t hi = hiIndex < span.textCapacity ? (uint8_t)span.text[hiIndex] : 0;
    const uint8_t lo = loIndex < span.textCapacity ? (uint8_t)span.text[loIndex] : 0;
    return (uint16_t)(((uint16_t)hi << 8) | lo);
}

bool ModbusDataMap::setSpanRegister(Binding& span, uint16_t offset, uint16_t value) {
    const uint16_t hiIndex = (uint16_t)(offset * 2);
    const uint16_t loIndex = (uint16_t)(hiIndex + 1);
    if (hiIndex < span.textCapacity) span.text[hiIndex] = (char)(value >> 8);
    if (loIndex < span.textCapacity) span.text[loIndex] = (char)(value & 0xFF);

    // The last byte stays a terminator whatever a client writes, so a peer
    // cannot turn a bound buffer into an unterminated one and make every later
    // read run off the end of it.
    span.text[span.textCapacity - 1] = '\0';
    return true;
}

// =============================================================================
// ACCESS
// =============================================================================

bool ModbusDataMap::readBit(ModbusTable table, uint16_t address, bool& out) const {
    const Binding* b = find(table, address);
    if (!b) return false;
    const uint16_t offset = (uint16_t)(address - b->start);

    switch (b->kind) {
        case Kind::Bits:
            out = *mb_elemAt<bool>(b->bits, offset, b->stride);
            return true;
        case Kind::BitHandler:
            out = b->bitReader(offset);
            return true;
        default:
            return false;   // A register binding is not a bit
    }
}

bool ModbusDataMap::readRegister(ModbusTable table, uint16_t address, uint16_t& out) const {
    const Binding* b = find(table, address);
    if (!b) return false;
    const uint16_t offset = (uint16_t)(address - b->start);

    switch (b->kind) {
        case Kind::Registers:
            out = *mb_elemAt<uint16_t>(b->registers, offset, b->stride);
            return true;
        case Kind::String:
            out = spanRegister(*b, offset);
            return true;
        case Kind::RegisterHandler:
            out = b->regReader(offset);
            return true;
        default:
            return false;
    }
}

bool ModbusDataMap::setBit(ModbusTable table, uint16_t address, bool value) {
    Binding* b = find(table, address);
    if (!b) return false;
    const uint16_t offset = (uint16_t)(address - b->start);

    switch (b->kind) {
        case Kind::Bits:
            *mb_elemAt<bool>(b->bits, offset, b->stride) = value;
            return true;
        case Kind::BitHandler:
            return b->bitWriter ? b->bitWriter(offset, value) : false;
        default:
            return false;
    }
}

bool ModbusDataMap::setRegister(ModbusTable table, uint16_t address, uint16_t value) {
    Binding* b = find(table, address);
    if (!b) return false;
    const uint16_t offset = (uint16_t)(address - b->start);

    switch (b->kind) {
        case Kind::Registers:
            *mb_elemAt<uint16_t>(b->registers, offset, b->stride) = value;
            return true;
        case Kind::String:
            return setSpanRegister(*b, offset, value);
        case Kind::RegisterHandler:
            return b->regWriter ? b->regWriter(offset, value) : false;
        default:
            return false;
    }
}

bool ModbusDataMap::writeBit(ModbusTable table, uint16_t address, bool value, uint8_t unit) {
    const Binding* b = find(table, address);
    if (!b || !b->writable()) return false;

    // Read the old value first, so the change handler is raised only when
    // something actually changed. A client writing the value that is already
    // there is not a change, and reporting it as one turns an event log into a
    // poll log.
    bool previous = false;
    const bool hadPrevious = readBit(table, address, previous);

    if (!setBit(table, address, value)) return false;

    countWrite(table, address);
    if (!hadPrevious || previous != value) {
        noteChange(table, address, value ? 1u : 0u, unit);
    }
    return true;
}

bool ModbusDataMap::writeRegister(ModbusTable table, uint16_t address, uint16_t value,
                                  uint8_t unit) {
    const Binding* b = find(table, address);
    if (!b || !b->writable()) return false;

    uint16_t previous = 0;
    const bool hadPrevious = readRegister(table, address, previous);

    if (!setRegister(table, address, value)) return false;

    countWrite(table, address);
    if (!hadPrevious || previous != value) {
        noteChange(table, address, value, unit);
    }
    return true;
}

// =============================================================================
// STRINGS
// =============================================================================

bool ModbusDataMap::setString(ModbusTable table, uint16_t address, const char* text) {
    Binding* b = find(table, address);
    if (!b || b->kind != Kind::String || b->start != address || !text) return false;

    // Cleared first, so a shorter string never leaves the tail of a longer one
    // behind it - which on a fault message reads as a fault that is still
    // present.
    memset(b->text, 0, b->textCapacity);

    const size_t usable = (size_t)b->textCapacity - 1;   // Last byte is the terminator
    size_t length = 0;
    while (length < usable && text[length] != '\0') length++;
    memcpy(b->text, text, length);
    b->text[b->textCapacity - 1] = '\0';
    return true;
}

bool ModbusDataMap::getString(ModbusTable table, uint16_t address, char* out,
                              uint16_t size) const {
    const Binding* b = find(table, address);
    if (!b || b->kind != Kind::String || b->start != address || !out || size == 0) {
        return false;
    }

    uint16_t i = 0;
    while (i + 1 < size && i < b->textCapacity && b->text[i] != '\0') {
        out[i] = b->text[i];
        i++;
    }
    out[i] = '\0';
    return true;
}

uint16_t ModbusDataMap::stringRegisters(ModbusTable table, uint16_t address) const {
    const Binding* b = find(table, address);
    if (!b || b->kind != Kind::String || b->start != address) return 0;
    return b->count;
}

// =============================================================================
// FAILSAFE
// =============================================================================

ModbusDataMap::SafeState* ModbusDataMap::findSafeState(ModbusTable table, uint16_t address) {
    for (SafeState& s : _safeStates) {
        if (s.table == table && s.address == address) return &s;
    }
    return nullptr;
}

const ModbusDataMap::SafeState* ModbusDataMap::findSafeState(ModbusTable table,
                                                             uint16_t address) const {
    for (const SafeState& s : _safeStates) {
        if (s.table == table && s.address == address) return &s;
    }
    return nullptr;
}

void ModbusDataMap::setFailsafe(ModbusTable table, uint16_t address,
                                ModbusFailsafe action, uint16_t safeValue) {
    if (SafeState* existing = findSafeState(table, address)) {
        existing->action = action;
        existing->value = safeValue;
        return;
    }
    SafeState s;
    s.table = table;
    s.address = address;
    s.action = action;
    s.value = safeValue;
    _safeStates.push(s);
}

void ModbusDataMap::setCoilFailsafe(uint16_t address, ModbusFailsafe action, bool safeValue) {
    setFailsafe(ModbusTable::Coil, address, action, safeValue ? 1u : 0u);
}

uint16_t ModbusDataMap::applyFailsafe() {
    uint16_t driven = 0;

    for (SafeState& s : _safeStates) {
        if (s.action != ModbusFailsafe::ResetToValue) continue;

        const bool ok = mb_tableIsBits(s.table)
            ? setBit(s.table, s.address, s.value != 0)
            : setRegister(s.table, s.address, s.value);
        if (!ok) continue;

        s.applied++;
        driven++;

        // Deliberately NOT the change handler. See the note on
        // ModbusFailsafeHandler: an application that logs only changes would
        // record every commanded action and miss every safety-driven one.
        if (_failsafeHandler) _failsafeHandler(s.table, s.address, s.value);
    }

    return driven;
}

uint16_t ModbusDataMap::failsafeCount(ModbusTable table, uint16_t address) const {
    const SafeState* s = findSafeState(table, address);
    return s ? s->applied : 0;
}

void ModbusDataMap::onFailsafeApplied(ModbusFailsafeHandler handler) {
    _failsafeHandler = handler;
}

// =============================================================================
// CHANGE NOTIFICATION
// =============================================================================

void ModbusDataMap::onChanged(ModbusTable table, uint16_t address, ModbusChangeHandler handler) {
    onChangedRange(table, address, 1, handler);
}

void ModbusDataMap::onChangedRange(ModbusTable table, uint16_t address, uint16_t count,
                                   ModbusChangeHandler handler) {
    if (!handler || count == 0) return;
    Watch w;
    w.table = table;
    w.start = address;
    w.count = count;
    w.handler = handler;
    if (!_watches.push(w)) return;

    // Room for every change one request could raise - at most one per watched
    // address - taken now, so a write never has to allocate to report itself.
    size_t watched = 0;
    for (const Watch& each : _watches) watched += each.count;
    _pendingChanges.reserve(watched < kMaxPendingChanges ? watched : kMaxPendingChanges);
}

bool ModbusDataMap::isWatched(ModbusTable table, uint16_t address) const {
    for (const Watch& w : _watches) {
        if (w.table != table) continue;
        if (address >= w.start && (uint32_t)address < (uint32_t)w.start + w.count) return true;
    }
    return false;
}

void ModbusDataMap::noteChange(ModbusTable table, uint16_t address, uint16_t value,
                               uint8_t unit) {
    // Only recorded when somebody is listening, so a map nobody watches never
    // allocates on the write path.
    if (!isWatched(table, address)) return;

    PendingChange c;
    c.table = table;
    c.address = address;
    c.value = value;
    c.unit = unit;
    _pendingChanges.push(c);
}

void ModbusDataMap::dispatchChangeEvents() {
    if (_pendingChanges.empty()) return;

    // Only the changes there now. A handler is application code and may write
    // to this map, which records more changes behind these; they are raised on
    // the next request, as they always were. Read by index and copied out,
    // because recording one may move the list.
    const size_t owed = _pendingChanges.size();
    for (size_t i = 0; i < owed; i++) {
        const PendingChange c = _pendingChanges[i];
        for (size_t k = 0; k < _watches.size(); k++) {
            const Watch& w = _watches[k];
            if (w.table != c.table) continue;
            if (c.address < w.start) continue;
            if ((uint32_t)c.address >= (uint32_t)w.start + w.count) continue;
            w.handler(c.table, c.address, c.value, c.unit);
        }
    }
    _pendingChanges.erase(0, owed);
}

// =============================================================================
// WRITE COUNTERS
// =============================================================================

uint32_t ModbusDataMap::bindingWrites(ModbusTable table, uint16_t address) const {
    const Binding* b = find(table, address);
    return b ? b->writes : 0;
}

void ModbusDataMap::countWrite(ModbusTable table, uint16_t address) {
    _totalWrites++;

    // Counted against the binding as well as the address, for a caller that
    // watches a range rather than one register.
    Binding* b = find(table, address);
    if (b) b->writes++;

    size_t oldest = 0;
    for (size_t i = 0; i < _writeCounts.size(); i++) {
        WriteCount& c = _writeCounts[i];
        if (c.table == table && c.address == address) {
            if (c.count != 0xFFFF) c.count++;   // Saturates rather than wrapping
            c.touched = ++_writeClock;
            return;
        }
        if (c.touched < _writeCounts[oldest].touched) oldest = i;
    }

    // Bounded, and evicting the LEAST RECENTLY WRITTEN.
    //
    // Without a bound this table grows one entry per distinct address a client
    // writes to. A device binding a block of two thousand registers, written
    // across by a SCADA package, ends up with two thousand entries and a two
    // thousand comparison scan on every write - memory and time both growing
    // with nothing the device can do about it, driven entirely by what a remote
    // peer chooses to write.
    //
    // Least-recently-written is the right thing to keep: the addresses anyone
    // polls a count for are command and setpoint registers, which are exactly
    // the ones being written repeatedly. A sweep across a block ages itself out
    // and leaves them alone.
    WriteCount c;
    c.table = table;
    c.address = address;
    c.count = 1;
    c.touched = ++_writeClock;

    if (_writeCounts.size() >= kMaxWriteCounts) {
        _writeCounts[oldest] = c;
        return;
    }
    // The whole table on the first write, so later ones never allocate.
    _writeCounts.reserve(kMaxWriteCounts);
    _writeCounts.push(c);
}

uint16_t ModbusDataMap::writeCount(ModbusTable table, uint16_t address) const {
    for (const WriteCount& c : _writeCounts) {
        if (c.table == table && c.address == address) return c.count;
    }
    return 0;
}

void ModbusDataMap::swap(ModbusDataMap& other) noexcept {
    _bindings.swap(other._bindings);
    _safeStates.swap(other._safeStates);
    ModbusFailsafeHandler handler = _failsafeHandler;
    _failsafeHandler = other._failsafeHandler;
    other._failsafeHandler = handler;
    _watches.swap(other._watches);
    _pendingChanges.swap(other._pendingChanges);
    _writeCounts.swap(other._writeCounts);
    uint32_t n = _writeClock;
    _writeClock = other._writeClock;
    other._writeClock = n;
    n = _totalWrites;
    _totalWrites = other._totalWrites;
    other._totalWrites = n;
}
