// This file sees the server and the point set only through ModbusServerSide
// and ModbusPointsPart, so nothing here can name their code and a node that
// only asks links none of it. Everything that creates one is in
// ModbusAPIServer.cpp.
#define MODBUS_API_WITHOUT_PARTS
#include "ModbusAPI.h"

#include <new>

#include <string.h>
#include "ModbusNoExcept.h"

// =============================================================================
// SHARED
// =============================================================================

ModbusAPI::ModbusAPI() noexcept
    : _client(_link), _poller(_client), _router(*this) {}

ModbusAPI::~ModbusAPI() {
    delete _points;
    dropServer();
}

void ModbusAPI::adopt(ModbusServerSide* side) {
    // Everything set before the server existed, so the order an application
    // calls things in never matters.
    _side = side;
    side->setIdentity(_identity);
    side->setFailsafeTimeout(_serverConfig.failsafeTimeout);
    if (_requestFilter) side->setFilter(_requestFilter);
    if (_eventHandler) side->onEvent(_eventHandler, _eventContext);
    if (_trace) side->onTrace(_trace, _traceContext);
}

void ModbusAPI::dropServer() {
    // A serial server that is not the active side was made by server() on a
    // node that listens; it goes too.
    if (_serial && _serial != _side) delete _serial;
    delete _side;
    _side = nullptr;
    _serial = nullptr;
    _serverOn = false;
}

void ModbusAPI::Router::onAdu(const ModbusAdu& adu) {
    // The client first. It can say DEFINITIVELY whether this frame answers a
    // request it has outstanding, where the server can only say whether the
    // unit id looks like ours - and on a bus with two masters those overlap.
    if (api._clientOn && api._client.handleAdu(adu)) return;
    if (api._serverOn && api._side && api._side->handleAdu(adu)) return;

    // Neither ours to answer nor an answer to us. On a shared bus this is
    // somebody else's traffic, which is entirely normal.
}

void ModbusAPI::Router::onFrameError(ModbusFrameError error, size_t bytes) {
    if (!api._eventHandler) return;

    switch (error) {
        case ModbusFrameError::Checksum:
            api._eventHandler(api._eventContext, ModbusEvent::ChecksumError, 0,
                              "a frame arrived corrupt");
            break;
        case ModbusFrameError::Noise:
            // Reported for a run rather than per byte: a floating RS-485 pair
            // produces this continuously, and an event per byte would bury
            // everything else.
            if (bytes >= 8) {
                api._eventHandler(api._eventContext, ModbusEvent::NoiseDiscarded, 0,
                                  "bytes discarded while hunting for a frame");
            }
            break;
        default:
            api._eventHandler(api._eventContext, ModbusEvent::FramingError, 0,
                              mb_frameErrorName(error));
            break;
    }
}

bool ModbusAPI::start(bool asClient) {
    _startError = nullptr;
    if (!_link.begin()) return false;

    if (asClient && !_clientOn) {
        // The queue's storage is taken here, once, so a figure the board
        // cannot hold is said now - not by every request coming back QueueFull.
        if (!_client.begin(_clientConfig)) {
            _link.end();
            _startError = "out of memory for the request queue";
            return false;
        }
        _clientOn = true;
    }
    _open = true;

    updateExpectation();
    return true;
}

void ModbusAPI::updateExpectation() {
    if (!_link.attached()) return;

    // What the codec should expect to decode - and on RTU this decides how
    // frames are found at all, so it is not bookkeeping.
    //
    // A node that only ANSWERS can work out a frame's length from its function
    // code and take exactly that many bytes, which is the framing that survives
    // a USB adapter, a pseudo-terminal or a busy loop. A node that also ASKS
    // hears both directions, where the same function code means two different
    // lengths, so it has to fall back on measuring silence - and silence is
    // exactly what a buffering medium destroys.
    //
    // So "am I a client" is answered by whether this node has ever actually
    // asked anybody for anything, not by whether a client object exists. A
    // plain RS-485 slave - the commonest thing anyone builds - then gets the
    // better framing without having to know any of this, and the moment it
    // does issue a request, the expectation widens on its own.
    const bool asks = _client.operations().hasIssued();

    ModbusExpect want;
    if (asks && _serverOn)  want = ModbusExpect::Either;
    else if (_serverOn)     want = ModbusExpect::Request;
    else                    want = ModbusExpect::Response;

    if (want == _expectation) return;
    _expectation = want;
    _link.codec().expect(want);
}

void ModbusAPI::poll() {
    // Cheap: a comparison, and a call into the codec only when the answer has
    // actually changed - which happens once, on the first request this node
    // ever makes.
    updateExpectation();

    maintainLink();

    _link.poll(_router);
    if (_serverOn && _side) _side->poll();
    if (_clientOn) _client.poll();
    _poller.poll();
    if (_clientOn && _points) _points->poll(_client);
}

void ModbusAPI::maintainLink() {
    // A listening server has no single link to keep up - its connections live
    // in the pool and come and go by themselves - and a node that was never
    // opened has nothing to re-open.
    if (!_open || !_link.attached()) return;

    if (_link.isUp()) {
        if (!_wasUp) {
            _wasUp = true;
            _reconnectAt = 0;

            // Counted when the link is actually UP, never when an attempt is
            // made. A TCP connect is non-blocking, so begin() reports success
            // the instant the syscall is accepted and the socket may still be
            // reaching for a host that is not there - counting that would
            // report hundreds of reconnects a second to an absent device.
            if (_everUp) {
                _reconnects++;

                // Everything becomes due at once. Holding to the old schedule
                // after a gap would leave a value reading stale for up to its
                // whole interval after the link is demonstrably back.
                _poller.refreshAll();
            }
            _everUp = true;
        }
        return;
    }

    if (_wasUp) {
        // The moment it went, not once per poll afterwards.
        _wasUp = false;
        if (_eventHandler) {
            _eventHandler(_eventContext, ModbusEvent::LinkDown, 0, "the link dropped");
        }
    }

    if (_reconnectMs == 0) return;

    const mb_time_t now = mb_millis();
    if (_reconnectAt != 0 && (int32_t)(now - _reconnectAt) < 0) return;
    _reconnectAt = now + _reconnectMs;

    if (_eventHandler) {
        _eventHandler(_eventContext, ModbusEvent::Reconnecting, 0, "");
    }

    // Whether this succeeds is not decided here: a non-blocking connect is
    // still reaching when it returns. The next poll that finds the link up is
    // what records the recovery.
    _link.begin();
}

void ModbusAPI::end() {
    _client.cancelAll();
    _link.end();
    _open = false;
}

bool ModbusAPI::connected() const {
    return const_cast<ModbusLink&>(_link).isUp();
}


// --- Client operations -----------------------------------------------------

ModbusResult ModbusAPI::readCoil(uint8_t unit, uint16_t address, bool& dest,
                                 ModbusDone done) {
    return _client.readCoil(unit, address, dest, done);
}
ModbusResult ModbusAPI::readCoils(uint8_t unit, uint16_t address, bool* dest,
                                  uint16_t count, ModbusDone done) {
    return _client.readCoils(unit, address, dest, count, done);
}
ModbusResult ModbusAPI::readIsts(uint8_t unit, uint16_t address, bool& dest,
                                 ModbusDone done) {
    return _client.readDiscreteInput(unit, address, dest, done);
}
ModbusResult ModbusAPI::readIsts(uint8_t unit, uint16_t address, bool* dest,
                                 uint16_t count, ModbusDone done) {
    return _client.readDiscreteInputs(unit, address, dest, count, done);
}
ModbusResult ModbusAPI::readHreg(uint8_t unit, uint16_t address, uint16_t& dest,
                                 ModbusDone done) {
    return _client.readHoldingRegister(unit, address, dest, done);
}
ModbusResult ModbusAPI::readHregs(uint8_t unit, uint16_t address, uint16_t* dest,
                                  uint16_t count, ModbusDone done) {
    return _client.readHoldingRegisters(unit, address, dest, count, done);
}
ModbusResult ModbusAPI::readIreg(uint8_t unit, uint16_t address, uint16_t& dest,
                                 ModbusDone done) {
    return _client.readInputRegister(unit, address, dest, done);
}
ModbusResult ModbusAPI::readIregs(uint8_t unit, uint16_t address, uint16_t* dest,
                                  uint16_t count, ModbusDone done) {
    return _client.readInputRegisters(unit, address, dest, count, done);
}

ModbusResult ModbusAPI::writeCoil(uint8_t unit, uint16_t address, bool value,
                                  ModbusDone done) {
    return _client.writeCoil(unit, address, value, done);
}
ModbusResult ModbusAPI::writeCoils(uint8_t unit, uint16_t address, const bool* values,
                                   uint16_t count, ModbusDone done) {
    return _client.writeCoils(unit, address, values, count, done);
}
ModbusResult ModbusAPI::writeHreg(uint8_t unit, uint16_t address, uint16_t value,
                                  ModbusDone done) {
    return _client.writeRegister(unit, address, value, done);
}
ModbusResult ModbusAPI::writeHregs(uint8_t unit, uint16_t address, const uint16_t* values,
                                   uint16_t count, ModbusDone done) {
    return _client.writeRegisters(unit, address, values, count, done);
}
ModbusResult ModbusAPI::maskHreg(uint8_t unit, uint16_t address, uint16_t andMask,
                                 uint16_t orMask, ModbusDone done) {
    return _client.maskWriteRegister(unit, address, andMask, orMask, done);
}
ModbusResult ModbusAPI::readWriteHregs(uint8_t unit,
                                       uint16_t readAddress, uint16_t* dest,
                                       uint16_t readCount,
                                       uint16_t writeAddress, const uint16_t* values,
                                       uint16_t writeCount, ModbusDone done) {
    return _client.readWriteRegisters(unit, readAddress, dest, readCount,
                                      writeAddress, values, writeCount, done);
}

ModbusResult ModbusAPI::readFloat(uint8_t unit, uint16_t address, float& dest,
                                  ModbusOrder order, ModbusDone done) {
    return _client.readFloat32(unit, address, dest, order, done);
}
ModbusResult ModbusAPI::readInt32(uint8_t unit, uint16_t address, int32_t& dest,
                                  ModbusOrder order, ModbusDone done) {
    return _client.readInt32(unit, address, dest, order, done);
}
ModbusResult ModbusAPI::readUInt32(uint8_t unit, uint16_t address, uint32_t& dest,
                                   ModbusOrder order, ModbusDone done) {
    return _client.readUInt32(unit, address, dest, order, done);
}
ModbusResult ModbusAPI::readDouble(uint8_t unit, uint16_t address, double& dest,
                                   ModbusOrder order, ModbusDone done) {
    return _client.readFloat64(unit, address, dest, order, done);
}
ModbusResult ModbusAPI::readInt64(uint8_t unit, uint16_t address, int64_t& dest,
                                  ModbusOrder order, ModbusDone done) {
    return _client.readInt64(unit, address, dest, order, done);
}

ModbusResult ModbusAPI::writeFloat(uint8_t unit, uint16_t address, float value,
                                   ModbusOrder order, ModbusDone done) {
    return _client.writeFloat32(unit, address, value, order, done);
}
ModbusResult ModbusAPI::writeInt32(uint8_t unit, uint16_t address, int32_t value,
                                   ModbusOrder order, ModbusDone done) {
    return _client.writeInt32(unit, address, value, order, done);
}
ModbusResult ModbusAPI::writeUInt32(uint8_t unit, uint16_t address, uint32_t value,
                                    ModbusOrder order, ModbusDone done) {
    return _client.writeUInt32(unit, address, value, order, done);
}
ModbusResult ModbusAPI::writeDouble(uint8_t unit, uint16_t address, double value,
                                    ModbusOrder order, ModbusDone done) {
    return _client.writeFloat64(unit, address, value, order, done);
}
ModbusResult ModbusAPI::writeInt64(uint8_t unit, uint16_t address, int64_t value,
                                   ModbusOrder order, ModbusDone done) {
    return _client.writeInt64(unit, address, value, order, done);
}

ModbusResult ModbusAPI::readText(uint8_t unit, uint16_t address, char* dest,
                                 uint16_t capacity, uint16_t registers, ModbusDone done) {
    return _client.readText(unit, address, dest, capacity, registers, done);
}
ModbusResult ModbusAPI::writeText(uint8_t unit, uint16_t address, const char* text,
                                  uint16_t registers, ModbusDone done) {
    return _client.writeText(unit, address, text, registers, done);
}

ModbusResult ModbusAPI::readDeviceId(uint8_t unit, uint8_t* dest, uint16_t capacity,
                                     ModbusDone done) {
    return _client.readDeviceIdentification(unit, 0x01, 0x00, dest, capacity, done);
}

ModbusResult ModbusAPI::ping(uint8_t unit, ModbusDone done) {
    // Diagnostics sub-function 0x0000, Return Query Data: whatever is sent
    // comes back unchanged. It proves the path end to end without touching a
    // single register, which is why it is the right thing to call a ping.
    static const uint8_t query[2] = { 0xA5, 0x37 };
    return _client.diagnostics(unit, 0x0000, query, sizeof query,
                               _pingEcho, sizeof _pingEcho, done);
}

// --- Watching --------------------------------------------------------------

size_t ModbusAPI::watchHreg(uint8_t unit, uint16_t address, uint16_t* dest,
                            uint16_t count, mb_time_t everyMs) {
    return _poller.addRegisters(unit, address, dest, count, everyMs,
                                ModbusTable::HoldingRegister);
}
size_t ModbusAPI::watchIreg(uint8_t unit, uint16_t address, uint16_t* dest,
                            uint16_t count, mb_time_t everyMs) {
    return _poller.addRegisters(unit, address, dest, count, everyMs,
                                ModbusTable::InputRegister);
}
size_t ModbusAPI::watchCoil(uint8_t unit, uint16_t address, bool* dest,
                            uint16_t count, mb_time_t everyMs) {
    return _poller.addBits(unit, address, dest, count, everyMs, ModbusTable::Coil);
}
size_t ModbusAPI::watchIsts(uint8_t unit, uint16_t address, bool* dest,
                            uint16_t count, mb_time_t everyMs) {
    return _poller.addBits(unit, address, dest, count, everyMs,
                           ModbusTable::DiscreteInput);
}
size_t ModbusAPI::watchFloat(uint8_t unit, uint16_t address, float* dest,
                             mb_time_t everyMs, ModbusOrder order) {
    return _poller.addFloat32(unit, address, dest, everyMs, order);
}

ModbusQuality ModbusAPI::quality(size_t watchHandle) const {
    return _poller.quality(watchHandle);
}
mb_time_t ModbusAPI::age(size_t watchHandle) const { return _poller.age(watchHandle); }
bool ModbusAPI::allGood() const { return _poller.allGood(); }
void ModbusAPI::deadband(size_t watchHandle, uint16_t amount) {
    _poller.setDeadband(watchHandle, amount);
}
void ModbusAPI::stopWatching(size_t watchHandle) { _poller.remove(watchHandle); }

// --- Events ----------------------------------------------------------------

void ModbusAPI::onChanged(ModbusPollChangeHandler handler) { _poller.onChange(handler); }

ModbusResult ModbusAPI::probeOrder(uint8_t unit, uint16_t address,
                                   ModbusOrderProbe& dest, ModbusDone done) {
    return _client.probeOrder(unit, address, dest, done);
}

void ModbusAPI::onEvent(ModbusEventHandler handler, void* context) {
    _eventHandler = handler;
    _eventContext = context;
    _client.onEvent(handler, context);
    if (_side) _side->onEvent(handler, context);
}

// --- Safety ----------------------------------------------------------------

void ModbusAPI::failsafeAfter(mb_time_t ms) {
    _serverConfig.failsafeTimeout = ms;
    if (_side) _side->setFailsafeTimeout(ms);
}

// --- Identity and access ---------------------------------------------------

void ModbusAPI::identity(const char* vendor, const char* product, const char* revision) {
    _identity.vendorName = vendor;
    _identity.productCode = product;
    _identity.majorMinorRevision = revision;
    if (_side) _side->setIdentity(_identity);
}
void ModbusAPI::onRequest(ModbusRequestFilter filter) {
    _requestFilter = filter;
    if (_side) _side->setFilter(filter);
}

// --- Diagnostics -----------------------------------------------------------

ModbusRequestState ModbusAPI::status(uint8_t unit, uint16_t address,
                                     ModbusResult& outcome) const {
    return _client.requestState(unit, address, outcome);
}
size_t ModbusAPI::pending() const { return _client.pendingCount(); }
bool ModbusAPI::idle() const { return _client.idle(); }
void ModbusAPI::cancelAll() { _client.cancelAll(); }
mb_time_t ModbusAPI::roundTrip(uint8_t unit) const { return _client.roundTripMs(unit); }
mb_time_t ModbusAPI::silenceFor(uint8_t unit) const { return _client.silenceFor(unit); }

ModbusLinkStats ModbusAPI::stats() const {
    ModbusLinkStats total = const_cast<ModbusLink&>(_link).stats();
    if (_side) total.add(_side->stats());
    return total;
}
void ModbusAPI::resetStats() {
    _link.resetStats();
    if (_side) _side->resetStats();
}

void ModbusAPI::onTrace(ModbusLink::TraceHandler handler, void* context) {
    _trace = handler;
    _traceContext = context;
    _link.onTrace(handler, context);
    if (_side) _side->onTrace(handler, context);
}

size_t ModbusAPI::clients() const { return _side ? _side->connectionCount() : 0; }

void ModbusAPI::timeout(mb_time_t ms) {
    _clientConfig.responseTimeout = ms;
    _client.config().responseTimeout = ms;
}
void ModbusAPI::retries(uint8_t count) {
    _clientConfig.maxRetries = count;
    _client.config().maxRetries = count;
}
void ModbusAPI::queueSize(size_t requests, size_t payloadBytes) {
    const ModbusClientConfig defaults;
    _clientConfig.queueCapacity = requests ? requests : defaults.queueCapacity;
    _clientConfig.payloadSpace = payloadBytes ? payloadBytes : defaults.payloadSpace;
}

// =============================================================================
// MODBUS RTU
// =============================================================================

ModbusRTU::ModbusRTU() noexcept
#if defined(MODBUS_PLATFORM_ARDUINO)
    : _stream(nullptr, 0)
#elif defined(MODBUS_PLATFORM_HOST)
    : _serial(ModbusSerialConfig())
#endif
{}

#if defined(MODBUS_PLATFORM_ARDUINO)
bool ModbusRTU::begin(Stream* serial, uint32_t baud) {
    _stream.setStream(serial, baud);
    _codec.setLineRate(baud);
    _link.attach(_stream, _codec);
    return start(true);
}

void ModbusRTU::directionPin(int pin, bool activeHigh, uint32_t holdUs) {
    _stream.setDirectionPin(pin, activeHigh, holdUs);
}
#endif

#if defined(MODBUS_PLATFORM_HOST)
bool ModbusRTU::begin(const char* device, uint32_t baud, ModbusParity parity) {
    ModbusSerialConfig config;
    config.device = device;
    config.baudRate = baud;
    config.parity = parity;
    _serial.setConfig(config);
    _codec.setLineRate(baud);
    _link.attach(_serial, _codec);
    return start(true);
}
#endif

bool ModbusRTU::begin(ModbusTransport& transport, uint32_t baudForTiming) {
    if (baudForTiming) _codec.setLineRate(baudForTiming);
    else _codec.setLineRate(transport.bitsPerSecond());
    _link.attach(transport, _codec);
    return start(true);
}


// =============================================================================
// MODBUS ASCII
// =============================================================================

ModbusASCII::ModbusASCII() noexcept
#if defined(MODBUS_PLATFORM_ARDUINO)
    : _stream(nullptr, 0)
#elif defined(MODBUS_PLATFORM_HOST)
    : _serial(ModbusSerialConfig())
#endif
{}

#if defined(MODBUS_PLATFORM_ARDUINO)
bool ModbusASCII::begin(Stream* serial, uint32_t baud) {
    _stream.setStream(serial, baud);
    _link.attach(_stream, _codec);
    return start(true);
}

void ModbusASCII::directionPin(int pin, bool activeHigh, uint32_t holdUs) {
    _stream.setDirectionPin(pin, activeHigh, holdUs);
}
#endif

#if defined(MODBUS_PLATFORM_HOST)
bool ModbusASCII::begin(const char* device, uint32_t baud, ModbusParity parity) {
    ModbusSerialConfig config;
    config.device = device;
    config.baudRate = baud;
    config.parity = parity;
    _serial.setConfig(config);
    _link.attach(_serial, _codec);
    return start(true);
}
#endif

bool ModbusASCII::begin(ModbusTransport& transport) {
    _link.attach(transport, _codec);
    return start(true);
}


// =============================================================================
// MODBUS TCP
// =============================================================================

ModbusTCP::ModbusTCP() noexcept {}

ModbusTCP::~ModbusTCP() {
    // The pool first: it touches the listener from its own destructor.
    dropServer();
    _pool = nullptr;
#if defined(MODBUS_PLATFORM_HOST)
    delete _ownListener;
    delete _ownTransport;
#endif
}

ModbusCodec& codecFor(ModbusTcpCodec& tcp, ModbusRtuCodec& rtu, bool rtuOverTcp) {
    return rtuOverTcp ? (ModbusCodec&)rtu : (ModbusCodec&)tcp;
}

#if defined(MODBUS_PLATFORM_HOST)
bool ModbusTCP::connect(const char* host, uint16_t port) {
    delete _ownTransport;
    _ownTransport = new (std::nothrow) ModbusTcpTransport(host, port);
    if (!_ownTransport) return false;      // Out of memory is a failed connect
    return connect(*_ownTransport);
}
#endif

bool ModbusTCP::connect(ModbusTransport& transport) {
    if (_rtuOverTcp) _rtuCodec.setFraming(ModbusFraming::Length);
    _link.attach(transport, codecFor(_codec, _rtuCodec, _rtuOverTcp));
    return start(true);
}



bool ModbusTCP::connected() const {
    if (_listener) return _listener->isListening();
    return ModbusAPI::connected();
}

const char* ModbusTCP::lastError() const {
    if (_startError) return _startError;
#if defined(MODBUS_PLATFORM_HOST)
    // A server first: a listen() that could not bind is the failure with an
    // answer worth reading, and a client that never connected has no transport
    // to ask.
    if (_ownListener) return _ownListener->lastError();
    if (_ownTransport) return _ownTransport->lastError();
#endif
    return "";
}


void ModbusTCP::poll() {
    if (_pool) _side->poll();

    // This override does NOT chain to ModbusAPI::poll(), so the link
    // maintenance has to be asked for here as well - a client half that never
    // reconnected because the base class's poll() was not the one running is
    // exactly the failure this whole thing exists to prevent.
    maintainLink();

    if (_link.attached()) {
        _link.poll(_router);
        if (_clientOn) _client.poll();
    }
    _poller.poll();
    if (_clientOn && _link.attached() && _points) _points->poll(_client);
}

void ModbusTCP::end() {
    _client.cancelAll();
    if (_pool) _side->end();
    if (_link.attached()) _link.end();
    _open = false;
}
