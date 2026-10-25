#include "ModbusAPI.h"

#include <new>
#include <string.h>
#include "ModbusNoExcept.h"

// =============================================================================
// MODBUS BRIDGE
// =============================================================================

ModbusBridge::ModbusBridge() noexcept
#if defined(MODBUS_PLATFORM_ARDUINO)
    : _stream(nullptr, 0)
#elif defined(MODBUS_PLATFORM_HOST)
    : _serial(ModbusSerialConfig())
#endif
{}

ModbusBridge::~ModbusBridge() {
    delete _gateway;
#if defined(MODBUS_PLATFORM_HOST)
    delete _ownListener;
#endif
}

#if defined(MODBUS_PLATFORM_HOST)
void ModbusBridge::serial(const char* device, uint32_t baud, ModbusParity parity) {
    ModbusSerialConfig config;
    config.device = device;
    config.baudRate = baud;
    config.parity = parity;
    _serial.setConfig(config);
    _codec.setLineRate(baud);
    _backLink.attach(_serial, _codec);
}
#endif

#if defined(MODBUS_PLATFORM_ARDUINO)
void ModbusBridge::serial(Stream* stream, uint32_t baud) {
    _stream.setStream(stream, baud);
    _codec.setLineRate(baud);
    _backLink.attach(_stream, _codec);
}

void ModbusBridge::directionPin(int pin, bool activeHigh, uint32_t holdUs) {
    _stream.setDirectionPin(pin, activeHigh, holdUs);
}
#endif

void ModbusBridge::serial(ModbusTransport& transport, uint32_t baudForTiming) {
    _codec.setLineRate(baudForTiming ? baudForTiming : transport.bitsPerSecond());
    _backLink.attach(transport, _codec);
}

#if defined(MODBUS_PLATFORM_HOST)
void ModbusBridge::listen(uint16_t port) {
    delete _ownListener;
    _ownListener = new (std::nothrow) ModbusTcpServer(port);
    // Left null on an exhausted heap. begin() then reports the failure, which
    // is where a caller is already checking.
    _listener = _ownListener;
}
#endif

void ModbusBridge::listen(ModbusListener& listener) {
    _listener = &listener;
}

bool ModbusBridge::begin() {
    // Both ends are required. A bridge with one is not half a bridge; it is a
    // configuration mistake worth reporting now rather than at the first
    // request.
    if (!_listener || !_backLink.attached()) return false;

    delete _gateway;
    _gateway = new (std::nothrow) ModbusGateway(*_listener, _backLink);
    if (!_gateway) return false;
    _gateway->setMaxConnections(_maxClients);
    _gateway->setMaxOutstanding(_maxOutstanding);
    _gateway->setIdentity(_identity);
    if (_eventHandler) _gateway->onEvent(_eventHandler, _eventContext);

    for (size_t i = 0; i < _routes.size(); i++) {
        _gateway->routeRange(_routes[i].first, _routes[i].last);
    }
    _gateway->routeEverythingElse(_catchAll);

    // Local bindings last, so a unit carved out of a routed range wins.
    for (size_t i = 0; i < _binds.size(); i++) applyBind(_binds[i]);

    if (!_gateway->begin()) {
        delete _gateway;
        _gateway = nullptr;
        return false;
    }
    return true;
}

void ModbusBridge::poll() {
    if (_gateway) _gateway->poll();
}

void ModbusBridge::end() {
    if (!_gateway) return;
    _gateway->end();
    delete _gateway;
    _gateway = nullptr;
}

void ModbusBridge::route(uint8_t unit) { route(unit, unit); }

void ModbusBridge::route(uint8_t first, uint8_t last) {
    const PendingRoute range = { first, last };
    _routes.push(range);
    if (_gateway) _gateway->routeRange(first, last);
}

void ModbusBridge::routeEverythingElse(bool enabled) {
    _catchAll = enabled;
    if (_gateway) _gateway->routeEverythingElse(enabled);
}

bool ModbusBridge::applyBind(const PendingBind& bind) {
    ModbusDataMap& target = _gateway->localMap(bind.unit);
    switch (bind.table) {
        case ModbusTable::Coil:
        case ModbusTable::DiscreteInput:
            return target.bindBlock(bind.table, bind.address,
                                    (bool*)bind.storage, bind.count);
        default:
            return target.bindBlock(bind.table, bind.address,
                                    (uint16_t*)bind.storage, bind.count);
    }
}

bool ModbusBridge::addCoils(uint8_t unit, uint16_t address, bool* array, uint16_t count) {
    const PendingBind bind = { unit, ModbusTable::Coil, address, count, (void*)array };
    _binds.push(bind);
    return _gateway ? applyBind(bind) : true;
}

bool ModbusBridge::addIsts(uint8_t unit, uint16_t address, bool* array, uint16_t count) {
    const PendingBind bind = { unit, ModbusTable::DiscreteInput, address, count, (void*)array };
    _binds.push(bind);
    return _gateway ? applyBind(bind) : true;
}

bool ModbusBridge::addHregs(uint8_t unit, uint16_t address, uint16_t* array, uint16_t count) {
    const PendingBind bind = { unit, ModbusTable::HoldingRegister, address, count, (void*)array };
    _binds.push(bind);
    return _gateway ? applyBind(bind) : true;
}

bool ModbusBridge::addIregs(uint8_t unit, uint16_t address, uint16_t* array, uint16_t count) {
    const PendingBind bind = { unit, ModbusTable::InputRegister, address, count, (void*)array };
    _binds.push(bind);
    return _gateway ? applyBind(bind) : true;
}

void ModbusBridge::identity(const char* vendor, const char* product, const char* revision) {
    _identity.vendorName = vendor;
    _identity.productCode = product;
    _identity.majorMinorRevision = revision;
    if (_gateway) _gateway->setIdentity(_identity);
}

void ModbusBridge::onEvent(ModbusEventHandler handler, void* context) {
    _eventHandler = handler;
    _eventContext = context;
    if (_gateway) _gateway->onEvent(handler, context);
}

void ModbusBridge::maxClients(size_t limit) {
    _maxClients = limit ? limit : 1;
    if (_gateway) _gateway->setMaxConnections(_maxClients);
}

void ModbusBridge::maxOutstanding(size_t limit) {
    _maxOutstanding = limit ? limit : 1;
    if (_gateway) _gateway->setMaxOutstanding(_maxOutstanding);
}

size_t   ModbusBridge::clients() const { return _gateway ? _gateway->connectionCount() : 0; }
uint32_t ModbusBridge::forwarded() const { return _gateway ? _gateway->forwarded() : 0; }
uint32_t ModbusBridge::servedLocally() const { return _gateway ? _gateway->servedLocally() : 0; }
uint32_t ModbusBridge::noPath() const { return _gateway ? _gateway->pathUnavailable() : 0; }
uint32_t ModbusBridge::deviceAbsent() const { return _gateway ? _gateway->targetFailed() : 0; }

ModbusGateway& ModbusBridge::gateway() {
    // begin() has not been called. Rather than hand back a reference to
    // nothing, build it now - the caller reaching past the facade is entitled
    // to an object, and this one is inert until begin().
    if (!_gateway && _listener && _backLink.attached()) {
        _gateway = new (std::nothrow) ModbusGateway(*_listener, _backLink);
    }
    return *_gateway;
}

ModbusDataMap& ModbusBridge::map(uint8_t unit) { return gateway().localMap(unit); }
