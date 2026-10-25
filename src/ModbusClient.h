#pragma once

#include "ModbusLink.h"
#include "ModbusOperations.h"


// =============================================================================
// THE CLIENT
// =============================================================================
//
// What used to be called a master. Reads and writes other devices' registers.
//
// NOTHING BLOCKS. Every call queues and returns; the answer arrives in the
// variable you pointed at, on a later poll(). One poll() in the application
// loop is the whole contract - the same code runs in an Arduino loop(), in a
// PLC scan task and in a Linux daemon.
//
// What the queue does for you, with no configuration at all, is described on
// ModbusOperations. The short version: duplicates are dropped, a repeated write
// sends only the value you settled on and TELLS whoever was waiting on the one
// it replaced, retries happen with backoff and jitter, and every accepted
// operation reaches exactly one conclusion.

struct ModbusClientConfig {
    // How long to wait for a response. MODBUS_AUTO measures it per unit, so a
    // VFD that answers in 4 ms and a soft-starter that takes 300 get the
    // timeout each deserves rather than one figure that suits neither.
    mb_time_t responseTimeout = MODBUS_AUTO;

    // How long to wait for a unit nothing has been measured about yet.
    //
    // MODBUS_AUTO is ten times the service-interval floor - generous, because
    // the cost of waiting too long ONCE is a slow first transaction, and the
    // cost of waiting too little is a device declared absent that was merely
    // thinking.
    //
    // Worth shortening when something downstream of this client is itself
    // waiting on us. A gateway does exactly that, and sets it lower so it can
    // answer a client before the client gives up - see ModbusGateway::begin.
    mb_time_t coldStartTimeout = MODBUS_AUTO;

    // Attempts after the first.
    uint32_t maxRetries = 2;

    // Backoff before a retry. MODBUS_AUTO derives it from the measured round
    // trip. Jittered either way, so two masters on one bus do not stay in step.
    mb_time_t retryDelay = MODBUS_AUTO;

    // Requests outstanding at once, per link.
    //
    // ONE on RTU and ASCII, and that is not a setting: serial Modbus carries no
    // transaction identifier, so two outstanding requests cannot be told apart
    // and the protocol forbids it. TCP may raise it, and many TCP devices still
    // cannot cope, which is why it stays opt-in.
    size_t maxInFlight = 1;

    // Silence after a broadcast before the next request goes out. Every server
    // on the segment is executing the write and none will answer, so there is
    // no other signal that they are done.
    mb_time_t turnaroundDelay = MODBUS_TURNAROUND_MS;

    // Deliberate pause between transactions, over and above the framing gap.
    // Some devices need one; most do not. Zero unless a device says otherwise.
    mb_time_t interFrameDelay = 0;

    // Operations that may be queued at once, and the bytes they may carry
    // between them (a write's values; a forwarded request). 64 and 16 KB on a
    // host, 16 and 1 KB on a board; see MODBUS_QUEUE_CAPACITY.
    size_t queueCapacity = MODBUS_QUEUE_CAPACITY;
    size_t payloadSpace = MODBUS_PAYLOAD_SPACE;
};

// Units a client keeps its own timing for. Every unit on a host; on a board,
// enough for a full field bus, and past that the unit heard from longest ago
// gives up its row.
#ifndef MODBUS_MAX_UNITS
    #if defined(MODBUS_PLATFORM_HOST)
        #define MODBUS_MAX_UNITS 248
    #else
        #define MODBUS_MAX_UNITS 16
    #endif
#endif

class ModbusClient {
public:
    explicit ModbusClient(ModbusLink& link) noexcept;

    // Takes the queue's storage. False when there is not memory for it; the
    // client then refuses every request with QueueFull.
    bool begin(const ModbusClientConfig& config = ModbusClientConfig());
    ModbusClientConfig& config() { return _config; }

    // Drives the client: expires what has timed out, and sends what is ready.
    // Does NOT drain the link - the owner does that and routes ADUs here, so
    // one link can carry a client and a server at once.
    void poll();

    // Offers a received ADU. Returns true when it answered one of our requests.
    //
    // False is normal and not an error: on a shared bus this may be another
    // master's traffic, or a reply that arrived after its own timeout while the
    // next request was already out. That late reply is the classic serial
    // corruption, and matching it to the new request would hand the application
    // one register's value under another's name.
    bool handleAdu(const ModbusAdu& adu);

    // =========================================================================
    // READS
    // =========================================================================
    //
    // `dest` is written when the answer arrives, and must stay valid until it
    // does. `stride` is the gap the caller's own layout puts between elements,
    // 0 for a plain array - which is what lets a response land straight in a
    // PLC's located-variable table with no copy in between.

    ModbusResult readCoils(uint8_t unit, uint16_t address, bool* dest, uint16_t count,
                           ModbusCompletionHandler onComplete = nullptr,
                           uint16_t stride = 0);
    ModbusResult readDiscreteInputs(uint8_t unit, uint16_t address, bool* dest,
                                    uint16_t count,
                                    ModbusCompletionHandler onComplete = nullptr,
                                    uint16_t stride = 0);
    ModbusResult readHoldingRegisters(uint8_t unit, uint16_t address, uint16_t* dest,
                                      uint16_t count,
                                      ModbusCompletionHandler onComplete = nullptr,
                                      uint16_t stride = 0);
    ModbusResult readInputRegisters(uint8_t unit, uint16_t address, uint16_t* dest,
                                    uint16_t count,
                                    ModbusCompletionHandler onComplete = nullptr,
                                    uint16_t stride = 0);

    // Single-value forms, for the common case.
    ModbusResult readCoil(uint8_t unit, uint16_t address, bool& dest,
                          ModbusCompletionHandler onComplete = nullptr);
    ModbusResult readDiscreteInput(uint8_t unit, uint16_t address, bool& dest,
                                   ModbusCompletionHandler onComplete = nullptr);
    ModbusResult readHoldingRegister(uint8_t unit, uint16_t address, uint16_t& dest,
                                     ModbusCompletionHandler onComplete = nullptr);
    ModbusResult readInputRegister(uint8_t unit, uint16_t address, uint16_t& dest,
                                   ModbusCompletionHandler onComplete = nullptr);

    // =========================================================================
    // WRITES
    // =========================================================================
    //
    // The values are COPIED when the call is made, so the caller's array may go
    // out of scope immediately. A write that held a pointer into the caller's
    // memory until the bus got round to it would be reading freed memory by
    // then - which, for a local, is the very next line.
    //
    // `unit` 0 is a broadcast: every server on the segment performs the write
    // and none of them answers. It concludes on transmission, and the client
    // then waits out the turnaround delay before its next request.

    ModbusResult writeCoil(uint8_t unit, uint16_t address, bool value,
                           ModbusCompletionHandler onComplete = nullptr);
    ModbusResult writeRegister(uint8_t unit, uint16_t address, uint16_t value,
                               ModbusCompletionHandler onComplete = nullptr);
    ModbusResult writeCoils(uint8_t unit, uint16_t address, const bool* values,
                            uint16_t count, ModbusCompletionHandler onComplete = nullptr,
                            uint16_t stride = 0);
    ModbusResult writeRegisters(uint8_t unit, uint16_t address, const uint16_t* values,
                                uint16_t count,
                                ModbusCompletionHandler onComplete = nullptr,
                                uint16_t stride = 0);

    // The only atomic read-modify-write Modbus has. Without it, two clients
    // setting different bits of one register corrupt each other: each reads,
    // modifies and writes back, and whichever writes second erases the first's
    // bit.
    ModbusResult maskWriteRegister(uint8_t unit, uint16_t address,
                                   uint16_t andMask, uint16_t orMask,
                                   ModbusCompletionHandler onComplete = nullptr);

    // Write then read, in ONE transaction. The specification is explicit that
    // the write happens first, so a client setting a command and reading back
    // the result gets the result of the command it just sent.
    ModbusResult readWriteRegisters(uint8_t unit,
                                    uint16_t readAddress, uint16_t* dest, uint16_t readCount,
                                    uint16_t writeAddress, const uint16_t* values,
                                    uint16_t writeCount,
                                    ModbusCompletionHandler onComplete = nullptr);

    // =========================================================================
    // VALUES WIDER THAN A REGISTER
    // =========================================================================
    //
    // A flow rate, a kilowatt-hour total, a position - none of them fit in
    // sixteen bits, so each spans two registers or four, and NOTHING ON THE
    // WIRE SAYS WHICH WAY ROUND.
    //
    // So the ordering is an explicit argument, named the way vendor manuals
    // name it. If a data sheet says "CDAB", pass CDAB. If it says nothing -
    // and they often say nothing - read two registers and call mb_probeOrder(),
    // which shows all four interpretations side by side; the one that is a
    // plausible flow rate is the device's.
    //
    // The conversion happens as the response is unpacked, straight into the
    // caller's variable. There is no staging buffer to keep alive.

    ModbusResult readFloat32(uint8_t unit, uint16_t address, float& dest,
                             ModbusOrder order = ModbusOrder::ABCD,
                             ModbusCompletionHandler onComplete = nullptr,
                             ModbusTable table = ModbusTable::HoldingRegister);
    ModbusResult readInt32(uint8_t unit, uint16_t address, int32_t& dest,
                           ModbusOrder order = ModbusOrder::ABCD,
                           ModbusCompletionHandler onComplete = nullptr,
                           ModbusTable table = ModbusTable::HoldingRegister);
    ModbusResult readUInt32(uint8_t unit, uint16_t address, uint32_t& dest,
                            ModbusOrder order = ModbusOrder::ABCD,
                            ModbusCompletionHandler onComplete = nullptr,
                            ModbusTable table = ModbusTable::HoldingRegister);
    ModbusResult readFloat64(uint8_t unit, uint16_t address, double& dest,
                             ModbusOrder order = ModbusOrder::ABCD,
                             ModbusCompletionHandler onComplete = nullptr,
                             ModbusTable table = ModbusTable::HoldingRegister);
    ModbusResult readInt64(uint8_t unit, uint16_t address, int64_t& dest,
                           ModbusOrder order = ModbusOrder::ABCD,
                           ModbusCompletionHandler onComplete = nullptr,
                           ModbusTable table = ModbusTable::HoldingRegister);

    // Arrays of wide values, laid end to end as devices publish them - a bank
    // of eight float32 totals, say. `stride` is the gap in the CALLER's layout.
    ModbusResult readFloat32Array(uint8_t unit, uint16_t address, float* dest,
                                  uint16_t count,
                                  ModbusOrder order = ModbusOrder::ABCD,
                                  ModbusCompletionHandler onComplete = nullptr,
                                  uint16_t stride = 0,
                                  ModbusTable table = ModbusTable::HoldingRegister);
    ModbusResult readInt32Array(uint8_t unit, uint16_t address, int32_t* dest,
                                uint16_t count,
                                ModbusOrder order = ModbusOrder::ABCD,
                                ModbusCompletionHandler onComplete = nullptr,
                                uint16_t stride = 0,
                                ModbusTable table = ModbusTable::HoldingRegister);
    ModbusResult readUInt32Array(uint8_t unit, uint16_t address, uint32_t* dest,
                                 uint16_t count,
                                 ModbusOrder order = ModbusOrder::ABCD,
                                 ModbusCompletionHandler onComplete = nullptr,
                                 uint16_t stride = 0,
                                 ModbusTable table = ModbusTable::HoldingRegister);
    ModbusResult readFloat64Array(uint8_t unit, uint16_t address, double* dest,
                                  uint16_t count,
                                  ModbusOrder order = ModbusOrder::ABCD,
                                  ModbusCompletionHandler onComplete = nullptr,
                                  uint16_t stride = 0,
                                  ModbusTable table = ModbusTable::HoldingRegister);
    ModbusResult readInt64Array(uint8_t unit, uint16_t address, int64_t* dest,
                                uint16_t count,
                                ModbusOrder order = ModbusOrder::ABCD,
                                ModbusCompletionHandler onComplete = nullptr,
                                uint16_t stride = 0,
                                ModbusTable table = ModbusTable::HoldingRegister);

    ModbusResult writeFloat32(uint8_t unit, uint16_t address, float value,
                              ModbusOrder order = ModbusOrder::ABCD,
                              ModbusCompletionHandler onComplete = nullptr);
    ModbusResult writeInt32(uint8_t unit, uint16_t address, int32_t value,
                            ModbusOrder order = ModbusOrder::ABCD,
                            ModbusCompletionHandler onComplete = nullptr);
    ModbusResult writeUInt32(uint8_t unit, uint16_t address, uint32_t value,
                             ModbusOrder order = ModbusOrder::ABCD,
                             ModbusCompletionHandler onComplete = nullptr);
    ModbusResult writeFloat64(uint8_t unit, uint16_t address, double value,
                              ModbusOrder order = ModbusOrder::ABCD,
                              ModbusCompletionHandler onComplete = nullptr);
    ModbusResult writeInt64(uint8_t unit, uint16_t address, int64_t value,
                            ModbusOrder order = ModbusOrder::ABCD,
                            ModbusCompletionHandler onComplete = nullptr);

    // Arrays of wide values, written as ONE request. The values are packed
    // into registers here, while the caller's array is known to be alive, so
    // the operation carries registers rather than a pointer back.
    //
    // One request rather than one per value is the whole point: eight float32
    // setpoints are eight round trips or one, and on a serial segment that is
    // the difference between a scan and a second.
    ModbusResult writeFloat32Array(uint8_t unit, uint16_t address, const float* values,
                                   uint16_t count,
                                   ModbusOrder order = ModbusOrder::ABCD,
                                   ModbusCompletionHandler onComplete = nullptr,
                                   uint16_t stride = 0);
    ModbusResult writeInt32Array(uint8_t unit, uint16_t address, const int32_t* values,
                                 uint16_t count,
                                 ModbusOrder order = ModbusOrder::ABCD,
                                 ModbusCompletionHandler onComplete = nullptr,
                                 uint16_t stride = 0);
    ModbusResult writeUInt32Array(uint8_t unit, uint16_t address, const uint32_t* values,
                                  uint16_t count,
                                  ModbusOrder order = ModbusOrder::ABCD,
                                  ModbusCompletionHandler onComplete = nullptr,
                                  uint16_t stride = 0);
    ModbusResult writeFloat64Array(uint8_t unit, uint16_t address, const double* values,
                                   uint16_t count,
                                   ModbusOrder order = ModbusOrder::ABCD,
                                   ModbusCompletionHandler onComplete = nullptr,
                                   uint16_t stride = 0);
    ModbusResult writeInt64Array(uint8_t unit, uint16_t address, const int64_t* values,
                                 uint16_t count,
                                 ModbusOrder order = ModbusOrder::ABCD,
                                 ModbusCompletionHandler onComplete = nullptr,
                                 uint16_t stride = 0);

    // The same two, for a caller that works its element type out at run time
    // rather than naming it - a binding behind a generic pin does exactly
    // that, and without these it would have to switch over every type twice.
    //
    // `stride` is the gap in the CALLER's layout, in bytes; 0 means the values
    // are packed. It exists because a PLC array element can carry more than
    // its value.
    ModbusResult readWideArray(uint8_t unit, uint16_t address, void* dest,
                               ModbusWideType type, ModbusOrder order, uint16_t count,
                               uint16_t stride = 0,
                               ModbusCompletionHandler onComplete = nullptr,
                               ModbusTable table = ModbusTable::HoldingRegister);
    ModbusResult writeWideArray(uint8_t unit, uint16_t address, const void* values,
                                ModbusWideType type, ModbusOrder order, uint16_t count,
                                uint16_t stride = 0,
                                ModbusCompletionHandler onComplete = nullptr);

    // Text packed two characters per register, first character in the high
    // byte. `capacity` is the destination's size in BYTES, terminator included.
    ModbusResult readText(uint8_t unit, uint16_t address, char* dest,
                          uint16_t capacity, uint16_t registers,
                          ModbusCompletionHandler onComplete = nullptr);
    ModbusResult writeText(uint8_t unit, uint16_t address, const char* text,
                           uint16_t registers,
                           ModbusCompletionHandler onComplete = nullptr);

    // =========================================================================
    // DIAGNOSTIC AND IDENTITY FUNCTIONS
    // =========================================================================

    ModbusResult readExceptionStatus(uint8_t unit, uint8_t& dest,
                                     ModbusCompletionHandler onComplete = nullptr);

    // Reads two holding registers and fills `dest` with all four word-order
    // interpretations of them, for commissioning a device whose data sheet is
    // silent on word order.
    ModbusResult probeOrder(uint8_t unit, uint16_t address, ModbusOrderProbe& dest,
                            ModbusCompletionHandler onComplete = nullptr);

    // Sub-function 0x0000 Return Query Data is the standard link test: whatever
    // is sent comes back unchanged, which proves the path end to end without
    // touching any register.
    ModbusResult diagnostics(uint8_t unit, uint16_t subFunction,
                             const uint8_t* data, uint16_t dataLen,
                             uint8_t* dest, uint16_t destCapacity,
                             ModbusCompletionHandler onComplete = nullptr);

    ModbusResult reportServerId(uint8_t unit, uint8_t* dest, uint16_t destCapacity,
                                ModbusCompletionHandler onComplete = nullptr);

    ModbusResult readDeviceIdentification(uint8_t unit, uint8_t readDeviceIdCode,
                                          uint8_t objectId,
                                          uint8_t* dest, uint16_t destCapacity,
                                          ModbusCompletionHandler onComplete = nullptr);

    ModbusResult readFifoQueue(uint8_t unit, uint16_t pointerAddress,
                               uint16_t* dest, uint16_t maxValues,
                               ModbusCompletionHandler onComplete = nullptr);

    // =========================================================================
    // PASSTHROUGH
    // =========================================================================

    // Sends these exact PDU bytes and hands back exactly what comes back.
    //
    // For a GATEWAY, which must carry a request it does not itself implement:
    // a device behind it may support function codes we do not, and re-encoding
    // from a decoded struct would quietly drop anything the struct cannot hold.
    //
    // Never deduplicated and never superseded - two identical requests from two
    // different clients are two different questions, and each is owed its own
    // answer on its own socket.
    ModbusResult forward(uint8_t unit, const uint8_t* pdu, uint16_t pduLen,
                         ModbusPassthroughHandler onResponse);

    // =========================================================================
    // STATE
    // =========================================================================

    // Where a request has got to, for a caller that polls rather than
    // registering a handler.
    ModbusRequestState requestState(uint8_t unit, uint16_t address,
                                    ModbusResult& result) const {
        return _ops.state(unit, address, result);
    }

    size_t pendingCount() const { return _ops.pendingCount(); }

    // The queue itself, for a caller that wants its diagnostics: how many
    // conclusions are being held for status(), how many were aged out, whether
    // any dispatch has had to carry work to the next poll. All bounded by
    // construction; all worth being able to see.
    ModbusOperations& operations() { return _ops; }
    const ModbusOperations& operations() const { return _ops; }
    bool idle() const { return _ops.empty(); }

    // Concludes everything outstanding, telling each caller it was cancelled.
    void cancelAll() { _ops.cancelAll(ModbusResult::Cancelled); }
    void cancelUnit(uint8_t unit) { _ops.cancelUnit(unit, ModbusResult::Cancelled); }

    // Measured round trip to a unit, in milliseconds, or 0 before anything has
    // been measured. What an integrator wants when asked "is that device slow
    // or is the bus slow".
    mb_time_t roundTripMs(uint8_t unit) const;

    // Milliseconds since this unit last answered anything, or
    // MB_NEVER_HEARD if it never has.
    //
    // The honest answer to "is that device there", and it costs nothing: it is
    // observed from traffic the application was making anyway rather than
    // generated by asking. A device polled every second and silent for thirty
    // is gone; one that was never heard from at all is a different problem, and
    // the sentinel keeps the two apart.
    mb_time_t silenceFor(uint8_t unit) const;

    // The response timeout currently in force for a unit.
    mb_time_t responseTimeoutMs(uint8_t unit) const;

    // PINS ONE UNIT'S RESPONSE TIMEOUT, whatever the client measures or was
    // configured with. For a device whose answers are slow by nature: behind
    // a LoRa mesh a reply takes one to three seconds, one hop or several, and
    // a measured timeout would either never get a first sample (the cold
    // start is far shorter) or collapse after a quick reply and declare the
    // next slow one lost. MODBUS_AUTO (0) hands the unit back to measuring.
    // The airtime of each exchange is still added on top, as for any unit.
    void setUnitTimeout(uint8_t unit, mb_time_t timeoutMs);
    mb_time_t unitTimeout(uint8_t unit) const;

    // The floor under every derived timeout: what this host can enforce, plus
    // what the medium physically needs. Neither can be measured away.
    mb_time_t timeoutFloorMs() const;

    void onEvent(ModbusEventHandler handler, void* context) {
        _eventHandler = handler;
        _eventContext = context;
    }

    ModbusLink& link() { return _link; }

private:
    ModbusLink& _link;
    ModbusOperations _ops;
    ModbusClientConfig _config;

    // Per-unit round-trip estimates. A sparse list rather than a 256-entry
    // array: a client usually talks to a handful of units, and 256 estimators
    // is two kilobytes an MCU does not have to spare.
    struct UnitTiming {
        uint8_t unit;
        ModbusEstimator estimator;

        // When this unit last answered anything. Zero until it has.
        mb_time_t lastHeard = 0;

        // setUnitTimeout(); MODBUS_AUTO while the unit is measured.
        mb_time_t pinnedMs = MODBUS_AUTO;
    };
    UnitTiming _timing[MODBUS_MAX_UNITS];
    size_t _timingCount = 0;
    ModbusEstimator* timingFor(uint8_t unit);
    void noteHeard(uint8_t unit, mb_time_t now);
    const ModbusEstimator* timingFor(uint8_t unit) const;

    // When the link may next be used, for the turnaround after a broadcast and
    // for a configured inter-frame delay.
    mb_time_t _holdUntil = 0;

    ModbusEventHandler _eventHandler = nullptr;
    void* _eventContext = nullptr;
    void raise(ModbusEvent event, uint8_t unit, const char* detail);

    // A wide read: the registers are converted into the caller's variable as
    // the response is unpacked.
    ModbusResult queueWide(uint8_t unit, uint16_t address, void* dest,
                           ModbusWideType type, ModbusOrder order, uint16_t count,
                           uint16_t stride, ModbusCompletionHandler onComplete,
                           ModbusTable table);

    // A wide write: the value is turned into registers here, while the caller's
    // variable is known to be alive.
    ModbusResult writeWide(uint8_t unit, uint16_t address, const uint8_t* wire,
                           uint16_t registers, ModbusCompletionHandler onComplete);

    // Queues one operation, having refused it locally where it can be.
    ModbusResult queue(uint8_t unit, const ModbusRequest& request,
                       const uint8_t* payload, uint16_t payloadLen,
                       void* dest, ModbusResultKind kind,
                       uint16_t destCapacity, uint16_t stride,
                       ModbusCompletionHandler onComplete);

    // Refuses an operation before it reaches the queue, and still reports it
    // exactly once. A caller that opens a slot per request must be told about a
    // refusal the same way it is told about a success.
    ModbusResult refuse(uint8_t unit, uint16_t address, ModbusResult why,
                        ModbusCompletionHandler onComplete);
};
