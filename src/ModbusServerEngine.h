#pragma once

#include "ModbusDataMap.h"
#include "ModbusPdu.h"

#include "ModbusCallback.h"
#include "ModbusArray.h"

// =============================================================================
// THE SERVER ENGINE
// =============================================================================
//
// Turns a request PDU into a response PDU against a data map. No I/O, no clock,
// no framing: the sessions carry the bytes and this decides what they mean.
//
// One engine serves every session, because what a server exposes does not
// depend on which client is asking. What IS per-session - framing, transaction
// state, which unit ids to answer for - lives in ModbusSession.

// Identity returned by 0x2B/0x0E Read Device Identification, and by 0x11 Report
// Server ID. How a commissioning tool tells one device from another, and worth
// filling in: the alternative is an integrator reading a register map to work
// out what they are looking at.
struct ModbusDeviceIdentity {
    const char* vendorName = nullptr;          // Object 0x00, mandatory
    const char* productCode = nullptr;         // Object 0x01, mandatory
    const char* majorMinorRevision = nullptr;  // Object 0x02, mandatory
    const char* vendorUrl = nullptr;           // Object 0x03, optional
    const char* productName = nullptr;         // Object 0x04, optional
    const char* modelName = nullptr;           // Object 0x05, optional
    const char* userApplicationName = nullptr; // Object 0x06, optional

    // 0x11 Report Server ID carries a device-specific byte string and a run
    // indicator. Serial line only.
    const uint8_t* serverIdBytes = nullptr;
    uint16_t serverIdLength = 0;
    bool running = true;
};

// Called before a request is applied. Return an exception code to refuse it, or
// MB_EX_NONE to let it through.
//
// This is where an application says what the protocol cannot: that writes are
// refused while the panel is in local mode, that one client may write and
// another may only read, that a command register only accepts values from a
// known set. Without it those rules have nowhere to live but inside the
// application's own change handler, by which point the write has already
// happened.
typedef ModbusCallback<uint8_t(uint8_t unit, const ModbusRequest& request)>
        ModbusRequestFilter;

// Serves a function code the library does not implement generically.
//
// Three of the public codes have no generic implementation, because what they
// address is device-specific by definition:
//
//   0x14 / 0x15  Read / Write File Record - a file store this library does not
//                have and should not invent
//   0x18         Read FIFO Queue - a queue belonging to the application
//
// And a device may implement vendor codes in 65..72 or 100..110, which the
// specification reserves for exactly that.
//
// Rather than refuse all of them for ever, an application can serve any code it
// likes. Write the response PDU BODY into `out` - the function code is written
// by the engine, so start at the first byte after it - and return its length.
// Return 0 and set `exception` to refuse; return 0 with `exception` left at
// MB_EX_NONE to answer with silence, which a broadcast or a listen-only mode
// needs.
//
// The request has already been decoded as far as its structure allows and has
// passed the access filter, so a handler sees the same view every built-in
// function does.
typedef ModbusCallback<size_t(uint8_t unit, const ModbusRequest& request,
                             uint8_t* out, size_t capacity, uint8_t& exception)>
        ModbusFunctionHandler;

// Counters for 0x08 Diagnostics and 0x0B Get Comm Event Counter, kept by the
// link and read here.
struct ModbusServerCounters {
    uint16_t busMessage = 0;          // 0x08/0x0B: messages seen on the bus
    uint16_t busCommError = 0;        // 0x08/0x0C: CRC failures
    uint16_t busExceptionError = 0;   // 0x08/0x0D: exception responses we sent
    uint16_t serverMessage = 0;       // 0x08/0x0E: messages addressed to us
    uint16_t serverNoResponse = 0;    // 0x08/0x0F: broadcasts, which we do not answer
    uint16_t serverNak = 0;           // 0x08/0x10
    uint16_t serverBusy = 0;          // 0x08/0x11
    uint16_t characterOverrun = 0;    // 0x08/0x12
    uint16_t eventCount = 0;          // 0x0B
    bool listenOnly = false;          // 0x08/0x04 puts us here; only 0x08/0x01 leaves
};

class ModbusServerEngine {
public:
    explicit ModbusServerEngine(ModbusDataMap& map) noexcept : _map(map) {}

    // The data map for a unit id. A plain server has one map and answers for
    // whatever unit it was configured with; a gateway has one per unit.
    ModbusDataMap& map() { return _map; }

    void setIdentity(const ModbusDeviceIdentity& identity) { _identity = identity; }
    const ModbusDeviceIdentity& identity() const { return _identity; }

    void setFilter(ModbusRequestFilter filter) { _filter = filter; }

    // Serves one function code from the application. Overrides a built-in
    // implementation where one exists, which is how a device with a real file
    // store or an unusual diagnostics set provides its own.
    void setFunctionHandler(uint8_t function, ModbusFunctionHandler handler);
    bool hasFunctionHandler(uint8_t function) const;

    ModbusServerCounters& counters() { return _counters; }
    const ModbusServerCounters& counters() const { return _counters; }

    // Whether this engine answers the serial-only function codes. False on TCP,
    // where their semantics - a shared bus, one device behind one address - do
    // not apply.
    void setSerialLine(bool serial) { _serialLine = serial; }
    bool isSerialLine() const { return _serialLine; }

    // --- The one entry point ---
    //
    // Builds the response to `pdu` into `out`. Returns the response length, or
    // 0 when NOTHING should be sent - a broadcast, or a request in listen-only
    // mode. The caller must distinguish those from a failure: on Modbus,
    // silence is a legitimate answer and an error is not.
    //
    // `unit` is the unit id the request was addressed to, carried through so a
    // change handler can say which of several units a write arrived for.
    size_t handle(uint8_t unit, bool broadcast,
                  const uint8_t* pdu, size_t pduLen,
                  uint8_t* out, size_t capacity);

private:
    ModbusDataMap& _map;
    ModbusDeviceIdentity _identity;
    ModbusRequestFilter _filter;

    // A sparse list rather than a 256-entry table: almost every server has
    // none of these, and paying 256 std::functions for the exception is the
    // wrong way round on a device with kilobytes.
    struct FunctionEntry {
        uint8_t function;
        ModbusFunctionHandler handler;
    };
    ModbusArray<FunctionEntry> _handlers;
    const ModbusFunctionHandler* findHandler(uint8_t function) const;

    ModbusServerCounters _counters;
    bool _serialLine = true;

    // One scratch buffer for a response payload, reused. A response is built
    // and sent before the next is begun, so one buffer is enough - and an
    // allocation per request on a device that answers ten thousand an hour for
    // years is how a long-running embedded server fragments its heap.
    uint8_t _scratch[MB_MAX_PDU];

    size_t refuse(uint8_t function, uint8_t exceptionCode, uint8_t* out, size_t capacity);

    uint8_t readBits(const ModbusRequest& req, ModbusTable table, uint8_t& byteCount);
    uint8_t readRegisters(const ModbusRequest& req, ModbusTable table, uint8_t& byteCount);
    uint8_t applyWrite(uint8_t unit, const ModbusRequest& req);

    size_t deviceIdentification(const ModbusRequest& req, uint8_t* out, size_t capacity,
                                uint8_t& refusal);

    // `silent` distinguishes "this sub-function is answered with nothing, by
    // design" from "this sub-function could not be built". Diagnostics has one
    // of each and they are not the same outcome: Force Listen Only Mode is
    // specified to send no response at all, where an unknown sub-function is an
    // illegal data value. Returning 0 for both would turn the first into the
    // second, and a client putting a device into listen-only would be told its
    // request was malformed.
    size_t diagnostics(const ModbusRequest& req, uint8_t* out, size_t capacity,
                       bool& silent);
    size_t reportServerId(uint8_t* out, size_t capacity);
};
