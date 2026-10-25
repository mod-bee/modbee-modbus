#include "ModbusAPI.h"

#include <new>
#include "ModbusNoExcept.h"

// =============================================================================
// THE PARTS A NODE CARRIES ONLY WHEN ASKED FOR
// =============================================================================
//
// Everything that names the server or the point set. ModbusAPI.cpp reaches
// them only through their interfaces (ModbusServerSide.h), so a node that never
// calls anything here links none of this - nor the server, data map, pool and
// point code it would pull in.

ModbusServer& ModbusAPI::server() {
    if (!_serial) {
        ModbusServer* serial = new (std::nothrow) ModbusServer(_link);
        if (!serial) {
            // Out of memory: a server that is never polled, so a caller holding
            // the reference does not fault. Its bindings simply go unserved.
            static ModbusLink spareLink;
            static ModbusServer spare(spareLink);
            return spare;
        }
        _serial = serial;
        if (!_side) adopt(serial);       // a node that listens keeps its pool
    }
    return *static_cast<ModbusServer*>(_serial);
}

ModbusServerSide& ModbusAPI::serverSide() {
    if (!_side) server();
    return _side ? *_side : server();
}

ModbusDataMap& ModbusAPI::map() { return serverSide().map(); }

ModbusPointSet& ModbusAPI::points() {
    if (!_points) _points = new (std::nothrow) ModbusPointSet();
    if (!_points) {
        static ModbusPointSet spare;     // out of memory, as for server()
        return spare;
    }
    return *static_cast<ModbusPointSet*>(_points);
}

// --- Binding ---------------------------------------------------------------

bool ModbusAPI::addCoil(uint16_t address, bool* variable) {
    return map().bind(ModbusTable::Coil, address, variable);
}
bool ModbusAPI::addIsts(uint16_t address, bool* variable) {
    return map().bind(ModbusTable::DiscreteInput, address, variable);
}
bool ModbusAPI::addHreg(uint16_t address, uint16_t* variable) {
    return map().bind(ModbusTable::HoldingRegister, address, variable);
}
bool ModbusAPI::addIreg(uint16_t address, uint16_t* variable) {
    return map().bind(ModbusTable::InputRegister, address, variable);
}

bool ModbusAPI::addCoils(uint16_t address, bool* array, uint16_t count) {
    return map().bindBlock(ModbusTable::Coil, address, array, count);
}
bool ModbusAPI::addIsts(uint16_t address, bool* array, uint16_t count) {
    return map().bindBlock(ModbusTable::DiscreteInput, address, array, count);
}
bool ModbusAPI::addHregs(uint16_t address, uint16_t* array, uint16_t count) {
    return map().bindBlock(ModbusTable::HoldingRegister, address, array, count);
}
bool ModbusAPI::addIregs(uint16_t address, uint16_t* array, uint16_t count) {
    return map().bindBlock(ModbusTable::InputRegister, address, array, count);
}

bool ModbusAPI::addString(uint16_t address, char* buffer, uint16_t capacity) {
    return map().bindString(ModbusTable::HoldingRegister, address, buffer, capacity);
}

bool ModbusAPI::addHregHandler(uint16_t address, uint16_t count,
                               ModbusRegisterReader reader, ModbusRegisterWriter writer) {
    return map().bindHandler(ModbusTable::HoldingRegister, address, count, reader, writer);
}
bool ModbusAPI::addIregHandler(uint16_t address, uint16_t count,
                               ModbusRegisterReader reader) {
    return map().bindHandler(ModbusTable::InputRegister, address, count, reader, nullptr);
}
bool ModbusAPI::addCoilHandler(uint16_t address, uint16_t count,
                               ModbusBitReader reader, ModbusBitWriter writer) {
    return map().bindHandler(ModbusTable::Coil, address, count, reader, writer);
}

bool ModbusAPI::remove(ModbusTable table, uint16_t address) {
    return map().unbind(table, address);
}

// --- Local access ----------------------------------------------------------

bool ModbusAPI::Coil(uint16_t address) const {
    bool value = false;
    const_cast<ModbusAPI*>(this)->map().readBit(ModbusTable::Coil, address, value);
    return value;
}
void ModbusAPI::Coil(uint16_t address, bool value) {
    map().setBit(ModbusTable::Coil, address, value);
}
bool ModbusAPI::Ists(uint16_t address) const {
    bool value = false;
    const_cast<ModbusAPI*>(this)->map().readBit(ModbusTable::DiscreteInput, address, value);
    return value;
}
void ModbusAPI::Ists(uint16_t address, bool value) {
    map().setBit(ModbusTable::DiscreteInput, address, value);
}
uint16_t ModbusAPI::Hreg(uint16_t address) const {
    uint16_t value = 0;
    const_cast<ModbusAPI*>(this)->map().readRegister(ModbusTable::HoldingRegister,
                                                     address, value);
    return value;
}
void ModbusAPI::Hreg(uint16_t address, uint16_t value) {
    map().setRegister(ModbusTable::HoldingRegister, address, value);
}
uint16_t ModbusAPI::Ireg(uint16_t address) const {
    uint16_t value = 0;
    const_cast<ModbusAPI*>(this)->map().readRegister(ModbusTable::InputRegister,
                                                     address, value);
    return value;
}
void ModbusAPI::Ireg(uint16_t address, uint16_t value) {
    map().setRegister(ModbusTable::InputRegister, address, value);
}

bool ModbusAPI::setString(uint16_t address, const char* text) {
    return map().setString(ModbusTable::HoldingRegister, address, text);
}
bool ModbusAPI::getString(uint16_t address, char* out, uint16_t size) const {
    return const_cast<ModbusAPI*>(this)->map()
        .getString(ModbusTable::HoldingRegister, address, out, size);
}

// --- Events ----------------------------------------------------------------

void ModbusAPI::onCoilChanged(uint16_t address, ModbusChangeHandler handler) {
    map().onChanged(ModbusTable::Coil, address, handler);
}
void ModbusAPI::onHregChanged(uint16_t address, ModbusChangeHandler handler) {
    map().onChanged(ModbusTable::HoldingRegister, address, handler);
}
void ModbusAPI::onHregChangedRange(uint16_t address, uint16_t count,
                                   ModbusChangeHandler handler) {
    map().onChangedRange(ModbusTable::HoldingRegister, address, count, handler);
}

// --- Safety ----------------------------------------------------------------

void ModbusAPI::failsafeCoil(uint16_t address, bool safeValue) {
    map().setCoilFailsafe(address, ModbusFailsafe::ResetToValue, safeValue);
}
void ModbusAPI::failsafeHreg(uint16_t address, uint16_t safeValue) {
    map().setFailsafe(ModbusTable::HoldingRegister, address,
                      ModbusFailsafe::ResetToValue, safeValue);
}
void ModbusAPI::onFailsafe(ModbusFailsafeHandler handler) {
    map().onFailsafeApplied(handler);
}

void ModbusRTU::slave(uint8_t unitId) {
    _serverConfig.unitId = unitId;
    _serverConfig.serialLine = true;
    ModbusServer& serial = server();
    serial.begin(_serverConfig);
    serial.setIdentity(_identity);
    _serverOn = true;
    updateExpectation();
}

void ModbusASCII::slave(uint8_t unitId) {
    _serverConfig.unitId = unitId;
    _serverConfig.serialLine = true;
    ModbusServer& serial = server();
    serial.begin(_serverConfig);
    serial.setIdentity(_identity);
    _serverOn = true;
    updateExpectation();
}

#if defined(MODBUS_PLATFORM_HOST)
bool ModbusTCP::listen(uint16_t port) {
    // THE POOL GOES FIRST. It holds a reference to the listener and touches it
    // from its own destructor, so freeing the listener first and the pool
    // second is a use-after-free - on the second listen() of an object's life,
    // which is exactly what a server does after a network reconfiguration.
    // What was bound to the registers is kept across it, as listen(listener)
    // keeps it, and handed to the new pool.
    ModbusDataMap kept;
    if (_side) kept.swap(_side->map());
    dropServer();
    _pool = nullptr;
    _listener = nullptr;

    delete _ownListener;
    _ownListener = new (std::nothrow) ModbusTcpServer(port);
    if (!_ownListener) return false;       // Out of memory is a failed listen
    const bool listening = listen(*_ownListener);
    if (_side) _side->map().swap(kept);    // the new pool's map is empty
    return listening;
}
#endif

bool ModbusTCP::listen(ModbusListener& listener) {
    // Again the pool first, for a caller that brings its own listener and
    // calls this twice. A serial server made by an earlier map() goes with it:
    // a listening node's registers are its pool's - but what was bound to
    // them is kept and handed to the new pool. A server listens again after a
    // network reconfiguration - an application re-opening it on a settings
    // change, say - and its registers must not vanish while every binding
    // still says it is bound.
    ModbusDataMap kept;
    if (_side) kept.swap(_side->map());
    dropServer();
    _pool = nullptr;

    _listener = &listener;
    ModbusServerPool* pool = new (std::nothrow) ModbusServerPool(
        listener, _rtuOverTcp ? ModbusSocketFraming::RtuOverTcp : ModbusSocketFraming::Tcp);
    if (!pool) return false;               // out of memory: the bindings go with `kept`
    _pool = pool;
    pool->map().swap(kept);

    // Unit 0 answers whatever unit id is asked for, which is right for TCP: a
    // client reaching us over TCP has already addressed us by IP, and the unit
    // id is only meaningful to a gateway.
    _serverConfig.unitId = 0;
    _serverConfig.serialLine = false;

    const bool begun = pool->begin(_serverConfig);

    // Anything set BEFORE listen() lives on this object; replay it onto the
    // pool that now owns the server side.
    adopt(pool);
    if (!begun) return false;
    pool->setEvictWhenFull(_displaceUnused);
    if (_maxClients) pool->setMaxConnections(_maxClients);
    if (_idleTimeoutSet) pool->setIdleTimeout(_idleTimeoutMs);
    _serverOn = true;
    _open = true;
    return true;
}

uint32_t ModbusTCP::refused() const { return _pool ? _pool->refusedCount() : 0; }
uint32_t ModbusTCP::displaced() const { return _pool ? _pool->evictedCount() : 0; }

void ModbusTCP::displaceUnusedConnections(bool enabled) {
    _displaceUnused = enabled;
    if (_pool) _pool->setEvictWhenFull(enabled);
}

void ModbusTCP::maxClients(size_t limit) {
    _maxClients = limit;
    if (_pool) _pool->setMaxConnections(limit);
}
void ModbusTCP::idleTimeout(mb_time_t ms) {
    _idleTimeoutMs = ms;
    _idleTimeoutSet = true;
    if (_pool) _pool->setIdleTimeout(ms);
}
