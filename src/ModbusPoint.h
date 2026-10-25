#pragma once

// =============================================================================
// A POINT DEFINED BY SETTINGS
// =============================================================================
//
// One value on a field device - a level, a flow, a relay - described entirely
// by data: unit, function code, address, data type, word order, bit, scale,
// offset, how often, when it counts as lost. Change any of it at run time and
// the point follows: the next read goes to the new address and the old answer
// is no longer trusted. That is what lets an installer swap a sensor model from
// a settings screen with no PLC change.
//
// A point is the poller's idea with three differences that matter in the field:
//
//   - IT IS RE-AIMED IN PLACE. configure() with a different address, unit or
//     type resets the quality to Unknown and discards a reply still on its way
//     for the old one. A watch registered once cannot do that.
//   - IT CARRIES ITS OWN RAW REGISTERS and decodes them, so INT16, UINT16,
//     INT32, UINT32, FLOAT32 and one bit of a register all go through one path
//     and come out as an engineering value (raw * scale + offset).
//   - IT WRITES TOO. FC05, FC06 and FC16 send the value when it changes and
//     again every `every` (a refresh, for a remote output with a watchdog).
//
// THE QUALITY IS THE POINT. Never read is Unknown, never Good with a zero. A
// value that stops arriving goes Uncertain, then Bad after `stale`, and the
// last good value is held - so a lost radio is never read as a valid 0 or a
// valid OFF.

#include "ModbusClient.h"
#include "ModbusPoller.h"   // ModbusQuality
#include "ModbusTyped.h"

#include "ModbusArray.h"
#include "ModbusServerSide.h"

// Ordinals match the ST enumerations MODBUS_PT_FC and MODBUS_PT_TYPE.
enum class ModbusPointFc : uint8_t {
    ReadCoils = 0,        // FC01
    ReadDiscreteInputs,   // FC02
    ReadHolding,          // FC03
    ReadInput,            // FC04
    WriteCoil,            // FC05
    WriteRegister,        // FC06
    WriteRegisters,       // FC16
};

enum class ModbusPointType : uint8_t {
    Bit = 0,       // a coil or discrete input (FC01/02/05)
    Int16,
    UInt16,
    Int32,         // two registers, in `order`
    UInt32,
    Float32,
    RegisterBit,   // bit `bit` (0..15) of one register, read only
};

const char* mb_pointTypeName(ModbusPointType type);

inline bool mb_pointFcWrites(ModbusPointFc fc) {
    return fc == ModbusPointFc::WriteCoil || fc == ModbusPointFc::WriteRegister ||
           fc == ModbusPointFc::WriteRegisters;
}

inline bool mb_pointFcBits(ModbusPointFc fc) {
    return fc == ModbusPointFc::ReadCoils || fc == ModbusPointFc::ReadDiscreteInputs ||
           fc == ModbusPointFc::WriteCoil;
}

// Registers a type occupies: 0 for a bit, 1 or 2 otherwise.
uint8_t mb_pointRegisters(ModbusPointType type);

// Whether this function code can carry this type. A coil is a bit; FC06 is one
// register; a register bit cannot be written without a read-modify-write.
bool mb_pointValid(ModbusPointFc fc, ModbusPointType type, uint8_t bit);

// Raw registers to the raw number, before scale and offset. False when the
// type needs more registers than `count`.
bool mb_pointDecode(const uint16_t* registers, uint8_t count, ModbusPointType type,
                    ModbusOrder order, uint8_t bit, double& raw);

// The raw number to registers, rounded and clamped to the type's range.
// Returns the registers used, 0 when the type cannot be written.
uint8_t mb_pointEncode(double raw, ModbusPointType type, ModbusOrder order,
                       uint16_t* registers);

struct ModbusPointConfig {
    uint8_t unit = 1;
    ModbusPointFc fc = ModbusPointFc::ReadHolding;
    uint16_t address = 0;
    ModbusPointType type = ModbusPointType::UInt16;
    ModbusOrder order = ModbusOrder::ABCD;
    uint8_t bit = 0;
    float scale = 1.0f;          // 0 is taken as 1: a scale of zero would read every value as 0
    float offset = 0.0f;
    mb_time_t everyMs = 1000;    // 0 = 1000
    mb_time_t staleMs = 0;       // 0 = 3 x everyMs
    mb_time_t timeoutMs = 0;     // this unit's response timeout; 0 = measured
    bool enabled = true;

    // Where it is and how it is read. A change of any of these is a different
    // point, and nothing learned about the old one carries over.
    bool sameTarget(const ModbusPointConfig& other) const {
        return unit == other.unit && fc == other.fc && address == other.address &&
               type == other.type && order == other.order && bit == other.bit;
    }
};

class ModbusPoint {
public:
    ModbusPoint() = default;
    ModbusPoint(const ModbusPoint&) = delete;
    ModbusPoint& operator=(const ModbusPoint&) = delete;

    // Applies a configuration. Cheap when nothing changed, so call it every
    // scan with the settings as they are now. Returns false (and the point
    // stands still with InvalidArgument) for a combination that cannot work:
    // a 32-bit type on FC06, a bit of a register on a write, a bit above 15.
    bool configure(const ModbusPointConfig& config);
    const ModbusPointConfig& config() const { return _config; }

    // For a write point: the value in engineering units. Sent when it differs
    // from what was last written, and again every everyMs.
    void setWriteValue(double value);

    // Issues a read or a write when one is due. Never blocks.
    void service(ModbusClient& client, mb_time_t now);

    // --- What it knows ---
    double value() const { return _value; }          // engineering units; held when not Good
    bool asBit() const { return _value != 0.0; }   // not bit(): Arduino.h defines that as a macro
    ModbusQuality quality(mb_time_t now) const;
    // Good or Uncertain: a value to act on. False before the first answer.
    bool ok(mb_time_t now) const;
    mb_time_t age(mb_time_t now) const;              // ModbusPoller::NEVER before the first answer
    uint16_t failures() const { return _failures; }
    ModbusResult lastResult() const { return _lastResult; }
    bool everGood() const { return _everGood; }
    uint32_t answers() const { return _answers; }    // successful transactions, ever
    bool inFlight() const { return _inFlight; }

private:
    void complete(uint32_t generation, ModbusResult result);
    static void completed(void* self, uintptr_t generation, ModbusResult result) {
        static_cast<ModbusPoint*>(self)->complete((uint32_t)generation, result);
    }
    mb_time_t interval() const;
    mb_time_t effectiveInterval() const;

    ModbusPointConfig _config;
    bool _configured = false;
    bool _valid = false;

    uint32_t _generation = 0;    // bumped on every re-aim; an older reply is discarded
    bool _inFlight = false;
    mb_time_t _lastAttemptMs = 0;
    bool _attempted = false;
    mb_time_t _lastGoodMs = 0;
    bool _everGood = false;
    uint16_t _failures = 0;
    ModbusResult _lastResult = ModbusResult::Queued;
    uint32_t _answers = 0;

    // Where a reply lands. Separate from _raw so a reply that arrives after a
    // re-aim cannot overwrite a value it does not describe.
    uint16_t _inbox[2] = {0, 0};
    bool _inboxBit = false;

    double _value = 0.0;
    double _writeValue = 0.0;
    double _written = 0.0;
    bool _hasWritten = false;
    double _sending = 0.0;
};

// The points one node services. Owned by the node (ModbusAPI::points()), so a
// point outlives every operation it queued: a reply concluding during the
// node's shutdown still lands in a live object.
class ModbusPointSet : public ModbusPointsPart {
public:
    // A run of `count` points, stable for the life of the set. Returns the
    // first index, or (size_t)-1 when there is no memory.
    ModbusPointSet() noexcept {}
    ~ModbusPointSet() override;
    size_t add(size_t count);
    ModbusPoint* at(size_t index);
    size_t count() const { return _points.size(); }

    // Services every point (one issue per point at most) and keeps each unit's
    // response timeout in step with the points that name one.
    void poll(ModbusClient& client) override;

    // Call after a configure() that changed a timeout, so poll() re-applies the
    // per-unit timeouts.
    void timeoutsChanged() { _timeoutsDirty = true; }

private:
    ModbusArray<ModbusPoint*> _points;    // each allocated once, by add()
    uint32_t _timedUnits[8] = { 0 };      // units whose timeout a point set
    size_t _cursor = 0;
    bool _timeoutsDirty = false;
};
