#pragma once

#include "ModbusClient.h"
#include "ModbusDataMap.h"
#include "ModbusLink.h"
#include "ModbusPoller.h"
#include "ModbusServerSide.h"
#include "codec/ModbusAsciiCodec.h"
#include "codec/ModbusRtuCodec.h"
#include "codec/ModbusTcpCodec.h"
#include "transport/ModbusSerialTransport.h"
#include "transport/ModbusStreamTransport.h"
#include "transport/ModbusTcpTransport.h"

#include "ModbusArray.h"

// Reached only through ModbusServerSide and ModbusPointsPart, and created only
// when asked for: a node that only asks never links them.
class ModbusServer;
class ModbusServerPool;
class ModbusPointSet;
class ModbusGateway;

// =============================================================================
// THE ONE-OBJECT API
// =============================================================================
//
// `ModbusRTU`, `ModbusASCII` and `ModbusTCP` are one object each, and between
// them they are the whole library for most people:
//
//     ModbusRTU mb;
//
//     void setup() {
//         Serial1.begin(19200, SERIAL_8E1);
//         mb.begin(&Serial1, 19200);
//         mb.slave(17);                    // answer as unit 17
//         mb.addHreg(40, &setpoint);       // this variable IS register 40
//     }
//
//     void loop() {
//         mb.poll();                       // the whole contract
//     }
//
// --- Nothing here is a wall ---
//
// Everything underneath stays public and reachable: `client()`, `server()`,
// `map()`, `link()`, `poller()`. This object composes those; it does not hide
// them. Start here, and reach past it the day you need something it does not
// offer - without rewriting what you already have.
//
// --- Names ---
//
// Coil, Ists, Hreg, Ireg - the four Modbus tables under the names every Modbus
// library and every device manual uses. `Ists` is a discrete input; `Ireg` is
// an input register. They are not our names and it would not help anyone to
// invent better ones.

// A completion handler, for a caller that wants to be told. Optional
// everywhere: `mb.readHreg(17, 100, value)` is a complete call.
typedef ModbusCompletionHandler ModbusDone;

// =============================================================================
// SHARED BEHAVIOUR
// =============================================================================
//
// Not used directly. ModbusRTU, ModbusASCII and ModbusTCP inherit it, and
// differ only in how they are opened.
class ModbusAPI {
public:
    virtual ~ModbusAPI();

    // =========================================================================
    // DRIVING
    // =========================================================================

    // Call this often - every time round your loop. It is the whole contract:
    // it reads what has arrived, answers what was asked of you, sends what you
    // queued, and times out what never came back.
    //
    // It never blocks, and never for longer than it takes to move the bytes
    // that are already there.
    virtual void poll();

    // Stops everything and closes the link. Anything still queued is concluded
    // as Cancelled, so nothing is left waiting.
    virtual void end();

    // Whether the link is usable right now. A TCP client reports false while
    // it is reconnecting; a TCP server reports whether it is listening, which
    // is the only sense the question has for something that has no single peer.
    virtual bool connected() const;

    // =========================================================================
    // BEING A SERVER - bind your own variables
    // =========================================================================
    //
    // These variables ARE the registers. The map holds their addresses and
    // reads and writes through them, so nothing copies and nothing can get out
    // of step. They must outlive this object - a global or a member, not a
    // local in a function that returns.
    //
    // Coils and holding registers are writable by a client. Discrete inputs
    // and input registers are not: there is no Modbus function code for
    // writing them, which is the protocol rather than a permission.

    bool addCoil(uint16_t address, bool* variable);
    bool addIsts(uint16_t address, bool* variable);       // discrete input
    bool addHreg(uint16_t address, uint16_t* variable);
    bool addIreg(uint16_t address, uint16_t* variable);   // input register

    // A run of addresses bound to an array, in ONE entry. This is the form to
    // reach for past a handful of registers: a thousand of them is one binding
    // and one lookup, where a thousand single bindings is a thousand of each.
    bool addCoils(uint16_t address, bool* array, uint16_t count);
    bool addIsts(uint16_t address, bool* array, uint16_t count);
    bool addHregs(uint16_t address, uint16_t* array, uint16_t count);
    bool addIregs(uint16_t address, uint16_t* array, uint16_t count);

    // Text across a run of registers, two characters each, NUL terminated.
    // `capacity` is the buffer's size in bytes; it occupies capacity / 2
    // registers.
    bool addString(uint16_t address, char* buffer, uint16_t capacity);

    // A range served by a function rather than by memory - a value computed on
    // demand, or read from hardware at the moment it is asked for. Leave the
    // writer out to make the range read-only.
    bool addHregHandler(uint16_t address, uint16_t count,
                        ModbusRegisterReader reader,
                        ModbusRegisterWriter writer = nullptr);
    bool addIregHandler(uint16_t address, uint16_t count, ModbusRegisterReader reader);
    bool addCoilHandler(uint16_t address, uint16_t count,
                        ModbusBitReader reader, ModbusBitWriter writer = nullptr);

    // Removes whatever binding starts at this address.
    bool remove(ModbusTable table, uint16_t address);

    // =========================================================================
    // YOUR OWN REGISTERS - reading and writing them locally
    // =========================================================================
    //
    // Usually unnecessary: you bound your variable, so just use it. These are
    // here for a register served by a handler, or where the binding is out of
    // scope.
    //
    // Setting a value locally is NOT a client write, so it raises no change
    // event - a change event reports somebody else acting on you.

    bool     Coil(uint16_t address) const;
    void     Coil(uint16_t address, bool value);
    bool     Ists(uint16_t address) const;
    void     Ists(uint16_t address, bool value);
    uint16_t Hreg(uint16_t address) const;
    void     Hreg(uint16_t address, uint16_t value);
    uint16_t Ireg(uint16_t address) const;
    void     Ireg(uint16_t address, uint16_t value);

    bool setString(uint16_t address, const char* text);
    bool getString(uint16_t address, char* out, uint16_t size) const;

    // =========================================================================
    // BEING A CLIENT - reading and writing somebody else
    // =========================================================================
    //
    // NOTHING BLOCKS. Every call queues and returns; the answer arrives in the
    // variable you pointed at, on a later poll().
    //
    // `dest` must stay valid until then. `unit` is the device's address.
    //
    // The value returned says whether the request was ACCEPTED, not whether it
    // succeeded - Queued means it is on its way. Pass a handler, or ask
    // `status()`, to learn how it ended.

    ModbusResult readCoil(uint8_t unit, uint16_t address, bool& dest, ModbusDone done = nullptr);
    ModbusResult readCoils(uint8_t unit, uint16_t address, bool* dest, uint16_t count,
                           ModbusDone done = nullptr);
    ModbusResult readIsts(uint8_t unit, uint16_t address, bool& dest, ModbusDone done = nullptr);
    ModbusResult readIsts(uint8_t unit, uint16_t address, bool* dest, uint16_t count,
                          ModbusDone done = nullptr);

    ModbusResult readHreg(uint8_t unit, uint16_t address, uint16_t& dest,
                          ModbusDone done = nullptr);
    ModbusResult readHregs(uint8_t unit, uint16_t address, uint16_t* dest, uint16_t count,
                           ModbusDone done = nullptr);
    ModbusResult readIreg(uint8_t unit, uint16_t address, uint16_t& dest,
                          ModbusDone done = nullptr);
    ModbusResult readIregs(uint8_t unit, uint16_t address, uint16_t* dest, uint16_t count,
                           ModbusDone done = nullptr);

    // Writes copy their values immediately, so the array you pass may go out of
    // scope the moment the call returns.
    //
    // Write the same address twice before the first has gone out and the queued
    // value is REPLACED, not stacked behind - so a control loop writing a
    // setpoint every scan sends the value it settled on. Whoever was waiting on
    // the replaced write is told `Superseded` rather than left hanging.
    ModbusResult writeCoil(uint8_t unit, uint16_t address, bool value,
                           ModbusDone done = nullptr);
    ModbusResult writeCoils(uint8_t unit, uint16_t address, const bool* values,
                            uint16_t count, ModbusDone done = nullptr);
    ModbusResult writeHreg(uint8_t unit, uint16_t address, uint16_t value,
                           ModbusDone done = nullptr);
    ModbusResult writeHregs(uint8_t unit, uint16_t address, const uint16_t* values,
                            uint16_t count, ModbusDone done = nullptr);

    // Sets and clears individual bits of one register ATOMICALLY. The only
    // read-modify-write Modbus has: without it, two clients setting different
    // bits of one register erase each other's.
    //
    //     result = (current AND andMask) OR (orMask AND NOT andMask)
    ModbusResult maskHreg(uint8_t unit, uint16_t address, uint16_t andMask,
                          uint16_t orMask, ModbusDone done = nullptr);

    // Writes and reads in ONE transaction, write first. For setting a command
    // and reading back its result without another device getting a turn in
    // between.
    ModbusResult readWriteHregs(uint8_t unit,
                                uint16_t readAddress, uint16_t* dest, uint16_t readCount,
                                uint16_t writeAddress, const uint16_t* values,
                                uint16_t writeCount, ModbusDone done = nullptr);

    // --- Values wider than a register -------------------------------------
    //
    // Nothing on the Modbus wire says which way round a 32-bit value is laid
    // out across its two registers, so the ordering is an argument - named the
    // way device manuals name it. If the manual does not say, read the two
    // registers and call `probeOrder()`, which shows all four readings at once.

    ModbusResult readFloat(uint8_t unit, uint16_t address, float& dest,
                           ModbusOrder order = ModbusOrder::ABCD,
                           ModbusDone done = nullptr);
    ModbusResult readInt32(uint8_t unit, uint16_t address, int32_t& dest,
                           ModbusOrder order = ModbusOrder::ABCD,
                           ModbusDone done = nullptr);
    ModbusResult readUInt32(uint8_t unit, uint16_t address, uint32_t& dest,
                            ModbusOrder order = ModbusOrder::ABCD,
                            ModbusDone done = nullptr);
    ModbusResult readDouble(uint8_t unit, uint16_t address, double& dest,
                            ModbusOrder order = ModbusOrder::ABCD,
                            ModbusDone done = nullptr);
    ModbusResult readInt64(uint8_t unit, uint16_t address, int64_t& dest,
                           ModbusOrder order = ModbusOrder::ABCD,
                           ModbusDone done = nullptr);

    ModbusResult writeFloat(uint8_t unit, uint16_t address, float value,
                            ModbusOrder order = ModbusOrder::ABCD,
                            ModbusDone done = nullptr);
    ModbusResult writeInt32(uint8_t unit, uint16_t address, int32_t value,
                            ModbusOrder order = ModbusOrder::ABCD,
                            ModbusDone done = nullptr);
    ModbusResult writeUInt32(uint8_t unit, uint16_t address, uint32_t value,
                             ModbusOrder order = ModbusOrder::ABCD,
                             ModbusDone done = nullptr);
    ModbusResult writeDouble(uint8_t unit, uint16_t address, double value,
                             ModbusOrder order = ModbusOrder::ABCD,
                             ModbusDone done = nullptr);
    ModbusResult writeInt64(uint8_t unit, uint16_t address, int64_t value,
                            ModbusOrder order = ModbusOrder::ABCD,
                            ModbusDone done = nullptr);

    // Reads two registers and hands back EVERY interpretation of them side by
    // side: all four word orders, as float, int32 and uint32. Look for the one
    // that is a plausible flow rate, and that is the device's ordering.
    //
    // The diagnostic that turns "the analogue reading is nonsense" from an
    // afternoon into a minute. `dest` must stay valid until it completes.
    ModbusResult probeOrder(uint8_t unit, uint16_t address, ModbusOrderProbe& dest,
                            ModbusDone done = nullptr);

    ModbusResult readText(uint8_t unit, uint16_t address, char* dest, uint16_t capacity,
                          uint16_t registers, ModbusDone done = nullptr);
    ModbusResult writeText(uint8_t unit, uint16_t address, const char* text,
                           uint16_t registers, ModbusDone done = nullptr);

    // --- Asking a device about itself --------------------------------------

    ModbusResult readDeviceId(uint8_t unit, uint8_t* dest, uint16_t capacity,
                              ModbusDone done = nullptr);

    // Sends whatever you give it and expects the same back. The standard link
    // test: it proves the path end to end without touching a single register.
    ModbusResult ping(uint8_t unit, ModbusDone done = nullptr);

    // =========================================================================
    // KEEPING A VALUE UP TO DATE
    // =========================================================================
    //
    // Say what to read and how often, and it lands in your variable on its own.
    // Returns a handle you can ask about later.
    //
    // The point is not saving you a timer. It is `quality()`: a variable
    // holding 1234 tells you nothing about whether 1234 is still true, and on a
    // bus where one device has quietly failed, a stale reading looks exactly
    // like a steady process.

    size_t watchHreg(uint8_t unit, uint16_t address, uint16_t* dest, uint16_t count,
                     mb_time_t everyMs);
    size_t watchIreg(uint8_t unit, uint16_t address, uint16_t* dest, uint16_t count,
                     mb_time_t everyMs);
    size_t watchCoil(uint8_t unit, uint16_t address, bool* dest, uint16_t count,
                     mb_time_t everyMs);
    size_t watchIsts(uint8_t unit, uint16_t address, bool* dest, uint16_t count,
                     mb_time_t everyMs);
    size_t watchFloat(uint8_t unit, uint16_t address, float* dest, mb_time_t everyMs,
                      ModbusOrder order = ModbusOrder::ABCD);

    // How much to believe a watched value.
    //
    //   Good       read within twice the interval you asked for - one
    //              interval's grace for the round trip
    //   Uncertain  up to four intervals old, or the last read failed once
    //   Bad        older than that, or two reads in a row have failed
    //   Unknown    never successfully read at all
    //
    // Unknown and Bad are different on purpose. One is a commissioning problem
    // - an address typed wrong, a device never fitted. The other is a running
    // fault - a cable, a device that lost power. An alarm list that conflates
    // them sends somebody to the wrong end of the plant.
    ModbusQuality quality(size_t watchHandle) const;

    // Milliseconds since that value last arrived.
    mb_time_t age(size_t watchHandle) const;

    // Whether every watched value is currently Good.
    bool allGood() const;

    // How much a value must move before onChanged() fires. Without one, an
    // analogue reading dithering in its last bit is an event on every read,
    // which is an event on nothing.
    void deadband(size_t watchHandle, uint16_t amount);

    void stopWatching(size_t watchHandle);

    // =========================================================================
    // BEING TOLD THINGS
    // =========================================================================

    // A client wrote one of YOUR registers. Raised once the whole request has
    // been applied, so you never see a half-written block, and only when the
    // value actually changed.
    void onCoilChanged(uint16_t address, ModbusChangeHandler handler);
    void onHregChanged(uint16_t address, ModbusChangeHandler handler);
    void onHregChangedRange(uint16_t address, uint16_t count, ModbusChangeHandler handler);

    // A watched value moved by more than its deadband.
    void onChanged(ModbusPollChangeHandler handler);

    // Something happened that is not one request's own outcome: a link came up
    // or went down, a frame arrived corrupt, a device refused something.
    void onEvent(ModbusEventHandler handler, void* context = nullptr);

    // =========================================================================
    // SAFETY
    // =========================================================================

    // What an output should do if the client stops talking to you.
    //
    // Worth setting on anything driving real plant. A pump commanded on by a
    // master that has since lost power stays on for ever otherwise, and nothing
    // in Modbus says it should not.
    //
    // Addresses with nothing set hold their last value, because driving an
    // output is an action and doing it unasked is its own hazard.
    void failsafeCoil(uint16_t address, bool safeValue);
    void failsafeHreg(uint16_t address, uint16_t safeValue);

    // How long the client may be silent before the safe states are applied.
    // Zero - the default - never applies them.
    void failsafeAfter(mb_time_t ms);

    // Called once per address driven to safety. Deliberately NOT the change
    // handler: that one reports a client acting, this one reports YOU acting
    // because a client stopped. Log both, or an incident review will show every
    // commanded action and no safety-driven one.
    void onFailsafe(ModbusFailsafeHandler handler);

    // =========================================================================
    // IDENTITY AND ACCESS
    // =========================================================================

    // What a commissioning tool sees when it asks what this device is.
    void identity(const char* vendor, const char* product, const char* revision);

    // Vet every request before it is applied. Return an MB_EX_* code to refuse
    // it, or MB_EX_NONE to let it through.
    //
    // This is where a panel in local mode refuses writes, and the only place it
    // can be said before the write has already happened.
    void onRequest(ModbusRequestFilter filter);

    // =========================================================================
    // FINDING OUT WHAT IS GOING ON
    // =========================================================================

    // Where a request has got to, for a caller that would rather ask than be
    // told. Idle, Pending, Done or Failed.
    ModbusRequestState status(uint8_t unit, uint16_t address, ModbusResult& outcome) const;

    // Requests queued and not yet concluded.
    size_t pending() const;

    // Clients currently connected to this node as a server.
    //
    // Zero for anything that is not a TCP server - a serial segment has a bus,
    // not a set of connections.
    size_t clients() const;
    bool idle() const;

    // Concludes everything outstanding as Cancelled, so nothing is left
    // waiting.
    void cancelAll();

    // Measured round trip to a device, in milliseconds. The number to quote
    // when somebody asks whether a device is slow or the bus is.
    mb_time_t roundTrip(uint8_t unit) const;

    // Milliseconds since this unit last answered anything, or MB_NEVER_HEARD.
    // Observed from traffic already happening, so asking costs nothing.
    mb_time_t silenceFor(uint8_t unit) const;

    // Which framing this link is using, and WHY, in a sentence.
    //
    // Virtual because the answer belongs to the codec and a caller holding the
    // base class cannot know which one it has - a wrapper written once for all
    // three framings is exactly that. "Why is my RTU splitting frames" is the most common question asked
    // of any Modbus library, and a node that cannot answer it is no use.
    //
    // Empty where framing is not a choice: a TCP link with an MBAP header has
    // its length in the header and nothing to decide.
    virtual const char* framingReason() const { return ""; }

    // Why the last begin(), connect() or listen() failed, in the operating
    // system's own words - "No such file or directory", "Address already in
    // use", "Permission denied".
    //
    // Text, because that is the whole value of it: a bool says the port did
    // not open and a code cannot say which of forty things went wrong at the
    // socket. Empty when nothing has failed, and empty on Arduino, where
    // opening a Stream the sketch already opened cannot fail this way.
    virtual const char* lastError() const { return _startError ? _startError : ""; }

    // How long to wait before trying a dropped link again, in milliseconds.
    //
    // A link that has gone MUST come back on its own: the far end of a Modbus
    // link is a device that gets rebooted, a switch that gets swapped, a USB
    // adapter that gets unplugged and put back. A client that stayed dead until
    // the application noticed would be a SCADA screen that never recovers from
    // somebody power-cycling a drive.
    //
    // Paced rather than continuous, because a failed connect to an absent host
    // costs a socket and a syscall, and doing that every poll() on a 1 ms task
    // is a thousand of each a second. Zero turns it off, for a caller that
    // would rather own the decision.
    void reconnectAfter(mb_time_t ms) { _reconnectMs = ms; }

    // Times this link has been re-opened after dropping. Worth a pin on a
    // diagnostic screen: a link that works but reconnects nightly is a
    // different problem from one that has never worked.
    uint32_t reconnects() const { return _reconnects; }

    // Frames sent and received, checksum errors, framing errors, noise. What to
    // look at first when a device will not answer: a rising checksum count is a
    // wiring or baud-rate problem, and a rising noise count is a floating bus.
    //
    // A TCP server is one link PER CONNECTION, not one link for its life, so
    // its connections are summed in - including those that have closed.
    ModbusLinkStats stats() const;
    void resetStats();

    // Every byte, both directions, as it goes past. What you actually want when
    // a device will not talk: not a decoded summary, the bytes.
    void onTrace(ModbusLink::TraceHandler handler, void* context = nullptr);

    // How long to wait for a reply. Left alone, it is measured per device - so
    // a slow drive and a fast I/O block on one bus each get the timeout they
    // deserve. Set it to pin one figure for everything.
    void timeout(mb_time_t ms);

    // Attempts after the first.
    void retries(uint8_t count);

    // How many requests may wait at once, and the bytes their values share (a
    // write's registers). Zero keeps the default: 16 and 1 KB on a board, 64
    // and 16 KB on a host. Taken once, when the link opens; a figure the board
    // cannot hold makes the open fail and lastError() says so, rather than
    // every request later coming back QueueFull.
    void queueSize(size_t requests, size_t payloadBytes = 0);

    // =========================================================================
    // REACHING PAST THIS OBJECT
    // =========================================================================
    //
    // Nothing here is hidden. These are the real objects; this class composes
    // them. Reach for them the day you need something the simple surface does
    // not offer, without rewriting what you already have.

    //
    // server(), map() and points() create what they return the first time they
    // are called (see ModbusServerSide.h). A TCP server's map is its
    // connection pool's, from listen() on.
    ModbusClient& client() { return _client; }
    ModbusServer& server();
    ModbusPoller& poller() { return _poller; }
    // Points defined by settings (ModbusPoint.h), serviced by poll().
    ModbusPointSet& points();
    ModbusLink& link() { return _link; }
    ModbusDataMap& map();

protected:
    ModbusAPI() noexcept;

    // Opens the link once a subclass has attached a transport and a codec.
    bool start(bool asClient);

    // Re-opens a link whose transport has dropped. Called from poll().
    void maintainLink();

    mb_time_t _reconnectMs = 2000;
    mb_time_t _reconnectAt = 0;
    uint32_t  _reconnects = 0;
    bool      _wasUp = false;
    bool      _everUp = false;

    // Tells the codec which directions this node actually hears. Re-evaluated
    // on every poll, and acts only when the answer changes.
    void updateExpectation();
    ModbusExpect _expectation = ModbusExpect::Either;

    // The parts created when first asked for. A pending operation of a point
    // is never concluded during shutdown - its handler is let go uncalled - so
    // the points may go first.
    ModbusPointsPart* _points = nullptr;
    ModbusServerSide* _side = nullptr;    // the server, once there is one
    ModbusServerSide* _serial = nullptr;  // a ModbusServer, when one exists

    // The server, created as a serial server if there is none yet; and taking
    // one on, with every setting made before it existed.
    ModbusServerSide& serverSide();
    void adopt(ModbusServerSide* side);
    void dropServer();

    ModbusLink _link;
    ModbusClient _client;
    ModbusPoller _poller;

    bool _clientOn = false;
    bool _serverOn = false;
    bool _open = false;

    ModbusClientConfig _clientConfig;
    ModbusServerConfig _serverConfig;

    // Why start() failed for a reason of this object's own rather than the
    // link's - the request queue it could not take. Null when it did not.
    const char* _startError = nullptr;

    // Held so identity() can be called before or after begin().
    ModbusDeviceIdentity _identity;

    // Routes what arrives to whichever half it belongs to.
    struct Router : public ModbusAduSink {
        explicit Router(ModbusAPI& owner) noexcept : api(owner) {}
        void onAdu(const ModbusAdu& adu) override;
        void onFrameError(ModbusFrameError error, size_t bytes) override;
        ModbusAPI& api;
    };
    Router _router;

    ModbusEventHandler _eventHandler = nullptr;
    void* _eventContext = nullptr;

    // Held for the same reason as _identity: it may be set before the object
    // that will enforce it exists.
    ModbusRequestFilter _requestFilter;
    ModbusLink::TraceHandler _trace = nullptr;
    void* _traceContext = nullptr;

    // Scratch for ping(), which must outlive the call that queued it.
    uint8_t _pingEcho[8] = { 0 };

};

// =============================================================================
// MODBUS RTU
// =============================================================================
//
// The usual choice on RS-485.
class ModbusRTU : public ModbusAPI {
public:
    ModbusRTU() noexcept;

#if defined(MODBUS_PLATFORM_ARDUINO)
    // Open the Stream yourself first - the baud rate and pins are board
    // specific. Pass the SAME baud rate here: Modbus RTU framing is defined in
    // character times, and it is the one thing that cannot be worked out from
    // the bus.
    bool begin(Stream* serial, uint32_t baud);

    // An RS-485 transceiver needs its driver enabled while transmitting. Call
    // this before begin(). `holdUs` is the pause after the last byte before the
    // driver is released - releasing too early cuts the final character in half,
    // and the far end reports a checksum error that points nowhere near the
    // transceiver.
    void directionPin(int pin, bool activeHigh = true, uint32_t holdUs = 2000);
#endif

#if defined(MODBUS_PLATFORM_HOST)
    // "/dev/ttyUSB0", a socat pseudo-terminal, or "COM3" on Windows.
    //
    // 8E1 is the Modbus default. Where parity is unavailable the specification
    // wants two stop bits, which ModbusParity::None selects - so it is 8N2, not
    // 8N1. A device set to 8N1 is off-spec, and ModbusParity::None8N1 exists
    // because such devices are real.
    bool begin(const char* device, uint32_t baud,
               ModbusParity parity = ModbusParity::Even);
#endif

    // Bring your own transport - a radio, a tunnel, anything with bytes.
    bool begin(ModbusTransport& transport, uint32_t baudForTiming = 0);

    // Also answer requests addressed to this unit. Without it, this node only
    // asks questions.
    void slave(uint8_t unitId);

    // Which framing is in use, and WHY, in a sentence. The first thing to read
    // when frames are going missing.
    const char* framingReason() const override { return _codec.framingReason(); }
    void forceFraming(ModbusFraming framing) { _codec.setFraming(framing); }

    const char* lastError() const override {
        if (_startError) return _startError;
#if defined(MODBUS_PLATFORM_HOST)
        return _serial.lastError();
#else
        return "";
#endif
    }

    ModbusRtuCodec& codec() { return _codec; }

private:
    ModbusRtuCodec _codec;
#if defined(MODBUS_PLATFORM_ARDUINO)
    ModbusStreamTransport _stream;
#endif
#if defined(MODBUS_PLATFORM_HOST)
    ModbusSerialTransport _serial;
#endif
};

// =============================================================================
// MODBUS ASCII
// =============================================================================
//
// For links that cannot hold RTU's character timing: a modem, a radio, a
// terminal server. Twice the bytes on the wire, and a full second allowed
// between characters.
class ModbusASCII : public ModbusAPI {
public:
    ModbusASCII() noexcept;

#if defined(MODBUS_PLATFORM_ARDUINO)
    bool begin(Stream* serial, uint32_t baud);
    void directionPin(int pin, bool activeHigh = true, uint32_t holdUs = 2000);
#endif
#if defined(MODBUS_PLATFORM_HOST)
    bool begin(const char* device, uint32_t baud,
               ModbusParity parity = ModbusParity::Even);
#endif
    bool begin(ModbusTransport& transport);

    void slave(uint8_t unitId);

    const char* framingReason() const override {
        return "ASCII framing: a frame runs from ':' to CRLF, so it needs no "
               "timing and no length - which is the whole reason to use it on a "
               "link that cannot hold RTU's character spacing";
    }

    const char* lastError() const override {
        if (_startError) return _startError;
#if defined(MODBUS_PLATFORM_HOST)
        return _serial.lastError();
#else
        return "";
#endif
    }

    ModbusAsciiCodec& codec() { return _codec; }

private:
    ModbusAsciiCodec _codec;
#if defined(MODBUS_PLATFORM_ARDUINO)
    ModbusStreamTransport _stream;
#endif
#if defined(MODBUS_PLATFORM_HOST)
    ModbusSerialTransport _serial;
#endif
};

// =============================================================================
// MODBUS TCP
// =============================================================================
//
// One object, two quite different jobs:
//
//     mb.connect("192.168.1.50");   // read somebody else
//     mb.listen();                  // let others read you
//
// A server faces several clients at once, each with its own socket. One
// register map serves all of them.
class ModbusTCP : public ModbusAPI {
public:
    ModbusTCP() noexcept;
    ~ModbusTCP() override;

    // --- As a client ---
#if defined(MODBUS_PLATFORM_HOST)
    bool connect(const char* host, uint16_t port = MODBUS_TCP_PORT);
#endif
    bool connect(ModbusTransport& transport);

    // --- As a server ---
#if defined(MODBUS_PLATFORM_HOST)
    bool listen(uint16_t port = MODBUS_TCP_PORT);
#endif
    bool listen(ModbusListener& listener);

    // How many clients to hold at once.
    //
    // When every slot is taken, a newcomer may displace a connection that has
    // NEVER ASKED FOR ANYTHING and has sat silent for several seconds - a port
    // scan, a half-open socket, a client that connected and died before its
    // first request. Those are sockets, not clients, and letting them hold a
    // device unreachable is the worse failure.
    //
    // A connection that has completed even one request is never displaced, no
    // matter how long it then goes quiet: thirty seconds between polls is an
    // ordinary client, not a dead one. When every slot is held by such a
    // client, a newcomer is accepted and immediately closed - which is a better
    // answer than a silent queue nobody will ever get to.
    void maxClients(size_t limit);

    // Close a connection nobody is using. A client that goes away without
    // closing leaves a socket that never reports an error.
    //
    // Sixty seconds by default. Zero disables it.
    void idleTimeout(mb_time_t ms);

    // Connections turned away because every slot was held by a working client,
    // and sockets displaced because they held a slot without ever using it.
    //
    // Worth putting on a diagnostic screen. A rising `refused` is the device
    // telling you maxClients() is too small for how many things want to talk to
    // it - which otherwise presents as "it works for everyone except me".
    uint32_t refused() const;
    uint32_t displaced() const;

    // Whether a newcomer may displace a socket that has never asked for
    // anything. On by default. Turn it off where the client set is fixed and an
    // unexpected connection should simply be refused.
    void displaceUnusedConnections(bool enabled);

    // An RTU frame inside a TCP stream - what every terminal server produces,
    // sharing port 502 with the real thing. Call before listen() or connect().
    void rtuOverTcp(bool enabled) { _rtuOverTcp = enabled; }

    const char* framingReason() const override {
        return _rtuOverTcp
            ? _rtuCodec.framingReason()
            : "MBAP framing: the header carries the length, so there is nothing "
              "to predict and nothing to measure";
    }

    void poll() override;
    void end() override;

    // For a server: whether it is listening. A server has no single peer, so
    // "is the link up" has no answer - and answering false while three clients
    // are connected is the wrong one.
    bool connected() const override;

    // A client's failure to connect, or a server's failure to bind. One object
    // does both jobs, so it reports whichever one was asked for.
    const char* lastError() const override;

private:
    ModbusTcpCodec _codec;
    ModbusRtuCodec _rtuCodec;
    bool _rtuOverTcp = false;

#if defined(MODBUS_PLATFORM_HOST)
    ModbusTcpTransport* _ownTransport = nullptr;
    ModbusTcpServer* _ownListener = nullptr;
#endif
    ModbusListener* _listener = nullptr;
    ModbusServerPool* _pool = nullptr;    // _side, once listen() has made it

    // Held so it can be set before listen() as well as after.
    bool _displaceUnused = true;

    // The same for these two: a node is usually configured before it listens,
    // and the pool that enforces them only exists from listen() on. 0 and
    // MODBUS_AUTO leave the pool's own defaults.
    size_t _maxClients = 0;
    mb_time_t _idleTimeoutMs = MODBUS_AUTO;
    bool _idleTimeoutSet = false;
};

// =============================================================================
// MODBUS BRIDGE - TCP in front, a serial segment behind
// =============================================================================
//
//      SCADA -+
//        HMI -+- TCP --> [ bridge ] --RS-485--> unit 1  unit 2  unit 17 ...
//     laptop -+
//
// The oldest and most useful box in the Modbus world. Several clients on the
// network side share one serial bus, which can only carry one conversation at
// a time; the bridge is what turns the second into the first.
//
//     ModbusBridge bridge;
//     bridge.serial("/dev/ttyUSB0", 19200);   // the bus behind
//     bridge.listen(502);                     // the clients in front
//     bridge.route(1, 32);                    // who is out there
//     bridge.begin();
//     while (running) bridge.poll();
//
// It does NOT decode what it carries. A device behind it may speak function
// codes this library has never heard of, and those pass through byte for byte.
//
// Two failures a bridge can report that a direct connection cannot:
//
//   0x0A Gateway Path Unavailable    we have no route to that unit, or no room
//                                    to start another transaction - a problem
//                                    with US.
//   0x0B Gateway Target Failed       the route exists and the device did not
//                                    answer - a problem with THE DEVICE.
//
// Worth wiring into an alarm separately. A plain timeout cannot tell an
// engineer which end of the plant to walk to.
class ModbusBridge {
public:
    ModbusBridge() noexcept;
    ~ModbusBridge();

    ModbusBridge(const ModbusBridge&) = delete;
    ModbusBridge& operator=(const ModbusBridge&) = delete;

    // --- The bus behind ---
    //
    // Configure it, then begin(). Nothing opens until then.

#if defined(MODBUS_PLATFORM_HOST)
    void serial(const char* device, uint32_t baud,
                ModbusParity parity = ModbusParity::Even);
#endif
#if defined(MODBUS_PLATFORM_ARDUINO)
    void serial(Stream* stream, uint32_t baud);
    void directionPin(int pin, bool activeHigh = true, uint32_t holdUs = 2000);
#endif
    // Any transport, for a bridge whose back end is not a local serial port -
    // a terminal server, a test harness. `baudForTiming` only sets RTU's
    // character timing; pass 0 to take it from the transport.
    void serial(ModbusTransport& transport, uint32_t baudForTiming = 0);

    // --- The clients in front ---

#if defined(MODBUS_PLATFORM_HOST)
    void listen(uint16_t port = MODBUS_TCP_PORT);
#endif
    void listen(ModbusListener& listener);

    // Opens both ends. Returns false if either will not open - a serial device
    // that is not there, a port already in use.
    bool begin();
    void poll();
    void end();

    // --- Routing ---

    void route(uint8_t unit);                       // this unit is out there
    void route(uint8_t first, uint8_t last);        // this range is out there

    // Forward anything not routed and not ours, rather than answering 0x0A.
    // For a bus whose devices you do not know in advance. Off by default,
    // because a typed-wrong unit id then becomes a timeout instead of an
    // immediate, honest "no such path".
    void routeEverythingElse(bool enabled = true);

    // --- The bridge's own registers ---
    //
    // A unit id carved out of the routed range, so a client can ask the bridge
    // about itself without a request reaching the field bus. A local binding
    // always wins over a route.

    bool addCoils(uint8_t unit, uint16_t address, bool* array, uint16_t count);
    bool addIsts(uint8_t unit, uint16_t address, bool* array, uint16_t count);
    bool addHregs(uint8_t unit, uint16_t address, uint16_t* array, uint16_t count);
    bool addIregs(uint8_t unit, uint16_t address, uint16_t* array, uint16_t count);

    void identity(const char* vendor, const char* product, const char* revision);
    void onEvent(ModbusEventHandler handler, void* context = nullptr);

    // How many clients to hold at once, and how many transactions may be
    // outstanding on the serial bus. The second is why a bridge is not just a
    // socket splice: eight clients asking at once still have to take turns.
    void maxClients(size_t limit);
    void maxOutstanding(size_t limit);

    // --- What it is doing ---

    size_t   clients() const;
    uint32_t forwarded() const;        // requests sent to the field bus
    uint32_t servedLocally() const;    // requests answered by the bridge itself
    uint32_t noPath() const;           // refused 0x0A - our problem
    uint32_t deviceAbsent() const;     // refused 0x0B - the device's problem

    // Which RTU framing the field bus is using, and why.
    const char* framingReason() const { return _codec.framingReason(); }
    void forceFraming(ModbusFraming framing) { _codec.setFraming(framing); }

    // --- The escape hatch ---
    ModbusGateway& gateway();
    ModbusDataMap& map(uint8_t unit);

private:
    ModbusRtuCodec _codec;
    ModbusLink     _backLink;
#if defined(MODBUS_PLATFORM_ARDUINO)
    ModbusStreamTransport _stream;
#endif
#if defined(MODBUS_PLATFORM_HOST)
    ModbusSerialTransport _serial;
    ModbusTcpServer*      _ownListener = nullptr;
#endif
    ModbusListener*    _listener = nullptr;
    ModbusGateway*     _gateway = nullptr;
    ModbusEventHandler _eventHandler = nullptr;
    void*              _eventContext = nullptr;
    ModbusDeviceIdentity _identity;

    struct PendingRoute { uint8_t first; uint8_t last; };
    ModbusArray<PendingRoute> _routes;
    bool   _catchAll = false;
    size_t _maxClients = 8;
    size_t _maxOutstanding = 8;

    // Bindings asked for before begin(), replayed once the gateway exists.
    struct PendingBind {
        uint8_t      unit;
        ModbusTable  table;
        uint16_t     address;
        uint16_t     count;
        void*        storage;
    };
    ModbusArray<PendingBind> _binds;

    bool applyBind(const PendingBind& bind);
};

// The classes behind server(), points() and ModbusBridge, for code that uses
// them through this header. ModbusAPI.cpp alone goes without (see
// ModbusServerSide.h).
#if !defined(MODBUS_API_WITHOUT_PARTS)
    #include "ModbusGateway.h"
    #include "ModbusPoint.h"
    #include "ModbusServer.h"
    #include "ModbusServerPool.h"
#endif
