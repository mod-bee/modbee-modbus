#pragma once

#include "ModbusClient.h"

#include "ModbusArray.h"

// =============================================================================
// THE POLL ENGINE
// =============================================================================
//
// Repeating reads into bound variables, with an honest answer to "should I
// still believe this".
//
// A Modbus client gives you a value. It does not tell you how old that value is,
// and a variable holding 21.5 says nothing about whether 21.5 is still true.
// That distinction is the whole difference between a number and a measurement,
// and it is what this layer adds:
//
//   Declare what to poll and how often     addPoll(...)
//   Values land in your own variables      no copying, no lookup
//   Ask whether to trust one               quality(), age()
//   Be told when it changes                onChange(), with a deadband
//
// Everything else in this library answers "what did that register say". This
// answers "what is that register, now".
//
// Everything else in this library answers "what did that register say". This
// answers "what IS that register, now" - which is a different question, and the
// one a control loop is actually asking.

// How much to believe a value.
enum class ModbusQuality : uint8_t {
    // Never successfully read. Distinct from Bad, which means it WAS read and
    // has since gone stale: one is a commissioning problem and the other is a
    // running fault, and an alarm list that conflates them is useless.
    Unknown = 0,

    // Read within twice its poll interval: one interval's grace, because a
    // poll issued on schedule still takes a round trip. Act on it.
    Good,

    // Late, but not by much - within four times the interval - or the last
    // attempt failed once. One missed poll on a busy bus looks like this and
    // is usually nothing.
    Uncertain,

    // Older than four intervals, or two attempts in a row have failed. Do
    // not act on it.
    Bad,
};

const char* mb_qualityName(ModbusQuality quality);

// A polled value changed. `index` is the poll's own, from addPoll's return.
typedef ModbusCallback<void(size_t index, uint16_t address)> ModbusPollChangeHandler;

// A poll's whole outcome, for a caller that wants the detail.
typedef ModbusCallback<void(size_t index, ModbusResult result)> ModbusPollHandler;

class ModbusPoller {
public:
    explicit ModbusPoller(ModbusClient& client) noexcept : _client(client) {}

    // =========================================================================
    // DECLARING WHAT TO POLL
    // =========================================================================
    //
    // Every form binds to the CALLER's own memory, which must outlive the
    // poller. The value lands there and nothing copies it afterwards - so an
    // application reads its own variable and never asks this class for anything
    // except quality.
    //
    // Returns an index, used to ask about that poll later. SIZE_MAX if refused.

    // `stride` is the gap in the CALLER's layout, in bytes; 0 means the values
    // are packed. It exists because an array element can carry more than its
    // value - a PLC's do - and without it a polled block lands in the wrong
    // slots and reads as plausible nonsense rather than as an error.
    size_t addRegisters(uint8_t unit, uint16_t address, uint16_t* dest, uint16_t count,
                        mb_time_t intervalMs, ModbusTable table = ModbusTable::HoldingRegister,
                        uint16_t stride = 0);
    size_t addBits(uint8_t unit, uint16_t address, bool* dest, uint16_t count,
                   mb_time_t intervalMs, ModbusTable table = ModbusTable::Coil,
                   uint16_t stride = 0);

    // A 32-bit or 64-bit value, in whichever word order the device uses.
    size_t addFloat32(uint8_t unit, uint16_t address, float* dest, mb_time_t intervalMs,
                      ModbusOrder order = ModbusOrder::ABCD);
    size_t addInt32(uint8_t unit, uint16_t address, int32_t* dest, mb_time_t intervalMs,
                    ModbusOrder order = ModbusOrder::ABCD);

    // One or many values wider than a register, for a caller that works its
    // element type out at run time rather than naming it.
    size_t addWide(uint8_t unit, uint16_t address, void* dest, ModbusWideType type,
                   uint16_t count, mb_time_t intervalMs,
                   ModbusOrder order = ModbusOrder::ABCD,
                   ModbusTable table = ModbusTable::HoldingRegister,
                   uint16_t stride = 0);

    // Stops polling one entry. The indices of the others do not change, so a
    // caller holding them keeps them.
    void remove(size_t index);

    // Starts it again, at the interval it already had. The inverse of
    // remove(), for a caller that watches a device only in some machine state
    // - without it, stopping and starting a watch would mean adding a new
    // entry every time and the table would grow for the life of the program.
    void resume(size_t index);

    // Changes how often an entry is polled, without disturbing its value, its
    // quality or its index.
    void setInterval(size_t index, mb_time_t intervalMs);

    // POINTS AN ENTRY SOMEWHERE ELSE, in place: another unit, address, table
    // or word order, into the same variable with the same count and type.
    // Nothing learned about the old address carries over - quality goes back
    // to Unknown, the failures and the deadband reference are cleared - and a
    // reply still on its way for the old address is discarded when it
    // concludes. The index stays valid and the table does not grow, so a
    // watch whose address comes from settings can follow them. False for an
    // index that does not exist, or a table the entry's kind cannot read
    // (bits from registers or the reverse).
    bool retarget(size_t index, uint8_t unit, uint16_t address, ModbusTable table,
                  ModbusOrder order);

    // Whether an entry is currently being polled.
    bool isActive(size_t index) const;

    void clear();

    size_t count() const { return _polls.size(); }

    // =========================================================================
    // TRUST
    // =========================================================================

    // How much to believe this poll's value, judged against the interval the
    // caller asked for - not against a figure this class invented.
    ModbusQuality quality(size_t index) const;

    // Milliseconds since this poll last succeeded, or MODBUS_NEVER.
    mb_time_t age(size_t index) const;
    static const mb_time_t NEVER = 0xFFFFFFFFu;

    // The last outcome, and how many times in a row it has failed. Consecutive
    // failures are what an alarm should be raised on: one is a bus hiccup, ten
    // is a device that has gone.
    ModbusResult lastResult(size_t index) const;
    uint16_t consecutiveFailures(size_t index) const;

    // Whether EVERY poll is currently Good. The one call a supervisory routine
    // usually wants.
    bool allGood() const;

    // =========================================================================
    // EVENTS
    // =========================================================================

    // Raised when a polled value changes by more than its deadband.
    //
    // The deadband is what makes this usable on a live plant: an analogue
    // reading dithers in its last bit for ever, and an event on every poll is
    // an event on nothing. Zero means report every change; the default is zero
    // because a coil has no meaningful deadband.
    void setDeadband(size_t index, uint16_t deadband);
    void onChange(ModbusPollChangeHandler handler) { _changeHandler = handler; }

    // Raised on every completion, successful or not. For a caller that wants to
    // see the failures as well as the values.
    void onPoll(ModbusPollHandler handler) { _pollHandler = handler; }

    // =========================================================================
    // DRIVING
    // =========================================================================

    // Issues whatever is due. Call it beside the client's own poll().
    //
    // Only ONE poll is issued per call, and only when the client has room.
    // A scan that queued everything at once would fill the operation queue with
    // work the bus cannot carry, and the last entry's value would be as old as
    // the whole scan by the time it arrived.
    void poll();

    // Marks every entry due immediately - after a reconnect, say, where holding
    // to the schedule would leave the first values as old as the outage.
    void refreshAll();

    // How far a failing entry's interval is allowed to stretch, as a multiple.
    //
    // A device that has gone must not keep asking at full rate. On a serial
    // segment only one transaction is in flight at a time, so every attempt on
    // an absent unit costs the WHOLE BUS a response timeout - and a handful of
    // dead devices can consume a segment entirely while the live ones go stale.
    //
    // So a failing entry backs off, doubling per consecutive failure up to this
    // multiple, and snaps back the instant it answers. Set to 1 to disable it,
    // where a device must be retried at full rate however long it has been
    // absent.
    void setFailureBackoff(uint16_t maxMultiple) {
        _maxBackoff = maxMultiple ? maxMultiple : 1;
    }

private:
    struct Poll {
        uint8_t unit = 0;
        uint16_t address = 0;
        uint16_t count = 0;
        ModbusTable table = ModbusTable::HoldingRegister;

        void* dest = nullptr;
        uint16_t stride = 0;   // Gap in the caller's layout, 0 for packed
        ModbusWideType wideType = ModbusWideType::None;
        ModbusOrder order = ModbusOrder::ABCD;
        bool bits = false;

        mb_time_t intervalMs = 1000;
        mb_time_t lastAttemptMs = 0;
        mb_time_t lastSuccessMs = 0;
        bool everSucceeded = false;
        bool inFlight = false;
        bool active = true;

        ModbusResult lastResult = ModbusResult::Queued;
        uint16_t failures = 0;
        uint16_t deadband = 0;

        // The previous value, for the deadband. Held for the first element
        // only: a deadband over a block is a question with no single answer,
        // and pretending otherwise would be worse than not offering it.
        uint32_t previous = 0;
        bool hasPrevious = false;
        // Reads still in flight that retarget() orphaned: each was for an old
        // address, and its conclusion must not count for the new one.
        uint8_t discard = 0;
    };

    ModbusClient& _client;
    ModbusArray<Poll> _polls;

    // Where the next scan starts, so one slow entry cannot starve the rest.
    size_t _cursor = 0;

    uint16_t _maxBackoff = 16;

    ModbusPollChangeHandler _changeHandler;
    ModbusPollHandler _pollHandler;

    size_t add(const Poll& poll);
    bool due(const Poll& poll, mb_time_t now) const;
    mb_time_t effectiveInterval(const Poll& poll) const;
    void issue(size_t index, mb_time_t now);
    void complete(size_t index, ModbusResult result);
    static void completed(void* self, uintptr_t index, ModbusResult result) {
        static_cast<ModbusPoller*>(self)->complete((size_t)index, result);
    }
    uint32_t snapshot(const Poll& poll) const;
};
