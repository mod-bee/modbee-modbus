#pragma once

#include "ModbusTypes.h"

#include "ModbusCallback.h"
#include "ModbusArray.h"

// =============================================================================
// THE DATA MAP
// =============================================================================
//
// What a server exposes: the four Modbus tables, bound to the application's own
// variables. No I/O and no clock; the server engine drives it and the sessions
// never see it.
//
// --- Bindings, not storage ---
//
// The map does not hold register values. It holds the ADDRESS of the caller's
// variable, and reads or writes through it for as long as the server runs. The
// application's `uint16_t level` IS holding register 30; nothing copies between
// them and nothing can get out of step.
//
// A bound variable must therefore OUTLIVE the map, or be unbound before it is
// destroyed. Static, global or member storage is the usual answer; a local in a
// function that returns, or an element of a container that reallocates, leaves a
// live pointer to memory that is gone, and nothing here can detect it.
//
// --- Four ways to bind ---
//
//   scalar    one address to one variable. Right for a sparse map: twelve
//             registers scattered across the address space.
//   block     a range of addresses to an array. Right for a PLC's contiguous
//             %IW / %QW areas, where a scalar binding per register would mean
//             ten thousand entries and a lookup through all of them.
//   string    text packed two characters to a register, NUL terminated.
//   handler   a range served by a callback, for values that are computed rather
//             than stored - a scaled reading, a bit assembled from elsewhere,
//             a register that must be read from hardware at the moment it is
//             asked for.
//
// All four are the same thing underneath: a run of addresses with something
// behind it. That is what makes overlap detection one function rather than six.

// What happens to an output when the master stops talking to us.
//
// A safe state is a property of the plant, so it is stated per address.
// HoldLastValue is the default, because driving an output is an action and
// doing it unasked is its own hazard.
enum class ModbusFailsafe : uint8_t {
    HoldLastValue = 0,
    ResetToValue,
};

// Serves a range of addresses from something other than a variable. `offset` is
// from the start of the bound range, not from address zero, so a handler does
// not have to know where it was bound.
//
// A read handler returns the value; a write handler returns false to refuse the
// write, which the engine reports as an illegal data value.
typedef ModbusCallback<uint16_t(uint16_t offset)>                 ModbusRegisterReader;
typedef ModbusCallback<bool(uint16_t offset, uint16_t value)>      ModbusRegisterWriter;
typedef ModbusCallback<bool(uint16_t offset)>                      ModbusBitReader;
typedef ModbusCallback<bool(uint16_t offset, bool value)>          ModbusBitWriter;

// A client wrote a new value into this map. Raised once the whole request has
// been applied, so a handler never sees a half-written block, and only when the
// value actually changed. `unit` is the unit id the request was addressed to.
typedef ModbusCallback<void(ModbusTable table, uint16_t address, uint16_t value,
                           uint8_t unit)> ModbusChangeHandler;

// An address was driven to its safe value because the master stopped talking.
//
// Deliberately NOT the change handler. That one reports a client writing a
// value; this one reports us acting because a client stopped. An application
// that centralises "an output changed, log it" on the change handler alone will
// record every commanded change and miss every safety-driven one, which is the
// wrong way round for an incident review. Register both.
typedef ModbusCallback<void(ModbusTable table, uint16_t address, uint16_t safeValue)>
        ModbusFailsafeHandler;

class ModbusDataMap {
public:
    ModbusDataMap() noexcept;
    ~ModbusDataMap();

    // Non-copyable: a copy would duplicate the bindings and leave two maps
    // writing through the same pointers, which is never what anyone means.
    ModbusDataMap(const ModbusDataMap&) = delete;
    ModbusDataMap& operator=(const ModbusDataMap&) = delete;

    // Exchanges everything with `other` - bindings, safe states, watches,
    // pending changes, write counts and the failsafe handler - so a server
    // that replaces its server side keeps what the application bound.
    void swap(ModbusDataMap& other) noexcept;

    // =========================================================================
    // BINDING
    // =========================================================================

    // One address to one variable. False if the pointer is null, the table is
    // the wrong shape for it, or the address is already covered.
    bool bind(ModbusTable table, uint16_t address, bool* variable);
    bool bind(ModbusTable table, uint16_t address, uint16_t* variable);

    // A run of addresses to an array of `count` elements. `stride` is the gap
    // the caller's own layout puts between elements, 0 for a plain array - so a
    // struct-of-arrays or a PLC's located-variable table can be bound directly
    // rather than copied into.
    bool bindBlock(ModbusTable table, uint16_t address, bool* array,
                   uint16_t count, uint16_t stride = 0);
    bool bindBlock(ModbusTable table, uint16_t address, uint16_t* array,
                   uint16_t count, uint16_t stride = 0);

    // Text over a run of registers: two characters each, big-endian, always NUL
    // terminated. The span occupies capacity / 2 registers, so the usable text
    // is registers * 2 - 1 characters. An odd capacity leaves its final byte
    // unused.
    //
    // Only the register tables. A string over coils would be one bit per two
    // characters, which is not a thing.
    bool bindString(ModbusTable table, uint16_t address, char* buffer, uint16_t capacity);

    // A run served by callbacks rather than by memory. Pass a null writer to
    // make the range read-only; a write to it is then refused as an illegal
    // data address, which is what a read-only holding register should look like
    // to a client.
    bool bindHandler(ModbusTable table, uint16_t address, uint16_t count,
                     ModbusRegisterReader reader, ModbusRegisterWriter writer = nullptr);
    bool bindHandler(ModbusTable table, uint16_t address, uint16_t count,
                     ModbusBitReader reader, ModbusBitWriter writer = nullptr);

    // Removes whatever binding starts at this address. The address is the start
    // of the binding, not any address inside it: half a block cannot be
    // unbound. False if nothing starts there.
    bool unbind(ModbusTable table, uint16_t address);

    // Drops every binding, every failsafe and every handler.
    void clear();

    // Bindings currently held, for diagnostics.
    size_t bindingCount() const { return _bindings.size(); }

    // =========================================================================
    // EXISTENCE
    // =========================================================================

    // Whether one address is bound.
    bool has(ModbusTable table, uint16_t address) const;

    // Whether EVERY address in the range is bound.
    //
    // The whole range, checked before any of it is touched. A read of ten
    // registers where three are unbound is one illegal data address, not a
    // partial response - and a block write that fails partway must write
    // nothing at all.
    bool hasRange(ModbusTable table, uint16_t address, uint16_t count) const;

    // Whether EVERY address in the range is bound AND accepts a write. A
    // handler bound without a writer is read-only, and a block write that
    // reaches one must be refused before anything is written.
    bool hasWritableRange(ModbusTable table, uint16_t address, uint16_t count) const;

    // =========================================================================
    // ACCESS
    // =========================================================================
    //
    // `unit` on the write paths is the unit id the request was addressed to,
    // carried through so a change handler can say which of several units a
    // write arrived for. It plays no part in deciding whether the write is
    // allowed.

    bool readBit(ModbusTable table, uint16_t address, bool& out) const;
    bool readRegister(ModbusTable table, uint16_t address, uint16_t& out) const;

    bool writeBit(ModbusTable table, uint16_t address, bool value, uint8_t unit = 0);
    bool writeRegister(ModbusTable table, uint16_t address, uint16_t value, uint8_t unit = 0);

    // The application's own way in, which does NOT raise change handlers: a
    // local set is the application writing to itself, and reporting it as a
    // client write would make a log of remote commands useless.
    bool setBit(ModbusTable table, uint16_t address, bool value);
    bool setRegister(ModbusTable table, uint16_t address, uint16_t value);

    // =========================================================================
    // STRINGS
    // =========================================================================

    // Writes text into a bound span. Text longer than the span holds is
    // truncated; the rest of the buffer is cleared either way, so a shorter
    // string never leaves the tail of a longer one behind it.
    bool setString(ModbusTable table, uint16_t address, const char* text);

    // Copies the text out, truncated to `size` and always terminated.
    bool getString(ModbusTable table, uint16_t address, char* out, uint16_t size) const;

    // Registers occupied by the span starting exactly at this address, or 0.
    uint16_t stringRegisters(ModbusTable table, uint16_t address) const;

    // =========================================================================
    // FAILSAFE
    // =========================================================================

    // What this address should do if the master stops talking. Addresses with
    // no failsafe set hold their last value, so a map that has never been told
    // about failsafe behaves as though it were off.
    void setFailsafe(ModbusTable table, uint16_t address, ModbusFailsafe action,
                     uint16_t safeValue = 0);
    void setCoilFailsafe(uint16_t address, ModbusFailsafe action, bool safeValue = false);

    // Drives every address with a stated safe state to it, and raises the
    // failsafe handler once per address. Called by the server when the master
    // has been silent past its timeout; safe to call directly.
    //
    // Returns how many addresses were driven.
    uint16_t applyFailsafe();

    // Times each address has been driven to its safe state, so a caller can
    // poll instead of registering a handler.
    uint16_t failsafeCount(ModbusTable table, uint16_t address) const;

    void onFailsafeApplied(ModbusFailsafeHandler handler);

    // =========================================================================
    // CHANGE NOTIFICATION
    // =========================================================================

    // Raised when a CLIENT writes a new value. Handlers are collected while a
    // request is applied and delivered once all of it has been, by
    // dispatchChangeEvents(), so a handler never sees a half-written block.
    //
    // Pass an address to watch one; the range form watches a run.
    void onChanged(ModbusTable table, uint16_t address, ModbusChangeHandler handler);
    void onChangedRange(ModbusTable table, uint16_t address, uint16_t count,
                        ModbusChangeHandler handler);

    // Delivers the changes collected since the last call. The engine calls this
    // once a request has been fully applied.
    void dispatchChangeEvents();

    // Writes a client has made to this address, as a running total, so a caller
    // can poll instead of registering.
    //
    // Held for the 64 most recently written addresses. That covers what this is
    // for - watching a command or setpoint register, which by definition is
    // written repeatedly - and keeps a client sweeping a large block from
    // growing the table without limit. An address that has aged out reads zero.
    uint16_t writeCount(ModbusTable table, uint16_t address) const;

    // Every write this map has accepted, whatever the address. Unlike the
    // per-address count this one cannot age out, so it is the honest answer to
    // "is anything writing to me at all".
    uint32_t totalWrites() const { return _totalWrites; }

    // Writes a client has made anywhere inside the BINDING that covers this
    // address, as a running total. Zero if nothing is bound there.
    //
    // This is what a caller watching a whole bound range wants, and neither of
    // the two above answers it: the per-address count ages out under a client
    // sweeping a large block, which is exactly when a range is interesting,
    // and the total counts writes to every OTHER binding as well - so two
    // callers watching two ranges would each see the other's traffic as their
    // own. One counter per binding is bounded by the bindings a program makes,
    // which it chose, rather than by what a remote peer decides to write.
    uint32_t bindingWrites(ModbusTable table, uint16_t address) const;

private:
    enum class Kind : uint8_t { Bits, Registers, String, BitHandler, RegisterHandler };

    // A run of addresses with something behind it. One shape for all four ways
    // of binding, which is what makes overlap detection a single function
    // instead of six pairwise ones.
    struct Binding {
        ModbusTable table = ModbusTable::Coil;
        Kind kind = Kind::Bits;
        uint16_t start = 0;
        uint16_t count = 0;          // addresses covered

        bool*     bits = nullptr;
        uint16_t* registers = nullptr;
        uint16_t  stride = 0;

        char*     text = nullptr;
        uint16_t  textCapacity = 0;

        // Writes a client has made anywhere in this range, monotonic.
        uint32_t writes = 0;

        ModbusRegisterReader regReader;
        ModbusRegisterWriter regWriter;
        ModbusBitReader      bitReader;
        ModbusBitWriter      bitWriter;

        // Whether this binding accepts a write. A handler range with no writer
        // is read-only; every other kind follows its table.
        bool writable() const;
    };

    // Sorted by (table, start), so a lookup is a binary search and an overlap
    // check is a look at the neighbour on each side. A sorted vector rather
    // than a map: no per-node allocation, and a block binding makes the entry
    // count small enough that the copy on insert never matters.
    ModbusArray<Binding> _bindings;

    const Binding* find(ModbusTable table, uint16_t address) const;
    Binding* find(ModbusTable table, uint16_t address);
    bool overlaps(ModbusTable table, uint16_t address, uint16_t count) const;
    bool insert(const Binding& binding);

    // --- Span helpers, shared by the string paths ---
    static uint16_t spanRegister(const Binding& span, uint16_t offset);
    static bool setSpanRegister(Binding& span, uint16_t offset, uint16_t value);

    // --- Failsafe storage ---
    struct SafeState {
        ModbusTable table;
        uint16_t address;
        ModbusFailsafe action;
        uint16_t value;
        uint16_t applied = 0;
    };
    ModbusArray<SafeState> _safeStates;
    SafeState* findSafeState(ModbusTable table, uint16_t address);
    const SafeState* findSafeState(ModbusTable table, uint16_t address) const;
    ModbusFailsafeHandler _failsafeHandler;

    // --- Change notification ---
    struct Watch {
        ModbusTable table;
        uint16_t start;
        uint16_t count;
        ModbusChangeHandler handler;
    };
    ModbusArray<Watch> _watches;

    struct PendingChange {
        ModbusTable table;
        uint16_t address;
        uint16_t value;
        uint8_t unit;
    };
    // Only changes with a handler bound are recorded, so a map nobody is
    // watching never allocates.
    ModbusArray<PendingChange> _pendingChanges;
    // The most a FC15 request can change: 1968 coils.
    static const size_t kMaxPendingChanges = 1968;
    bool isWatched(ModbusTable table, uint16_t address) const;
    void noteChange(ModbusTable table, uint16_t address, uint16_t value, uint8_t unit);

    // --- Write counters ---
    struct WriteCount {
        ModbusTable table;
        uint16_t address;
        uint16_t count;
        uint32_t touched;   // Monotonic, for least-recently-written eviction
    };

    // Holds the most recently written addresses, not every address ever
    // written. See countWrite() for why a bound is not optional here.
    static const size_t kMaxWriteCounts = 64;
    ModbusArray<WriteCount> _writeCounts;
    uint32_t _writeClock = 0;
    uint32_t _totalWrites = 0;
    void countWrite(ModbusTable table, uint16_t address);
};
