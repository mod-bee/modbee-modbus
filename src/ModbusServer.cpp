#include "ModbusServer.h"

#include <new>
#include "ModbusNoExcept.h"

ModbusServer::ModbusServer(ModbusLink& link) noexcept : _link(link), _engine(_map) {}

ModbusServer::~ModbusServer() {
    for (size_t i = 0; i < _unitMaps.size(); i++) delete _unitMaps[i].map;
}

void ModbusServer::begin(const ModbusServerConfig& config) {
    _config = config;
    _engine.setSerialLine(config.serialLine);
    _lastRequestAt = mb_millis();
    _heardFromMaster = false;
    _failsafeApplied = false;
}

ModbusDataMap& ModbusServer::mapFor(uint8_t unit) {
    // A broadcast is addressed to everybody, so it is addressed to US, and it
    // acts on our own map. Falling through to the per-unit lookup would
    // conjure an empty map for unit 0 and silently apply every broadcast to
    // nothing.
    if (unit == MB_ADDRESS_BROADCAST) return _map;

    if (unit == _config.unitId || _config.unitId == 0) return _map;
    for (size_t i = 0; i < _unitMaps.size(); i++) {
        if (_unitMaps[i].unit == unit) return *_unitMaps[i].map;
    }
    UnitMap entry;
    entry.unit = unit;
    // nullptr on an exhausted heap, where the build has no exceptions. Falling
    // back to the default map is wrong in principle and right in practice: it
    // keeps the device answering rather than faulting, and the alternative is
    // a null dereference in the request path.
    entry.map = new (std::nothrow) ModbusDataMap();
    if (!entry.map) return _map;
    _unitMaps.push(entry);
    return *entry.map;
}

bool ModbusServer::hasMapFor(uint8_t unit) const {
    if (unit == _config.unitId || _config.unitId == 0) return true;
    for (size_t i = 0; i < _unitMaps.size(); i++) {
        if (_unitMaps[i].unit == unit) return true;
    }
    return false;
}

bool ModbusServer::addressedToUs(uint8_t unit) const {
    // A broadcast is for everybody.
    if (unit == MB_ADDRESS_BROADCAST) return true;

    // Unit 0 configured means "answer whatever you are asked", which is right
    // for TCP: a client reaching us over TCP has already addressed us by IP,
    // and the unit id is then only meaningful to a gateway.
    if (_config.unitId == 0) return true;

    if (unit == _config.unitId) return true;

    // A per-unit map claims that unit as well, which is what makes a gateway
    // possible without a second server object per downstream device.
    return hasMapFor(unit);
}

mb_time_t ModbusServer::silenceMs() const {
    if (!_heardFromMaster) return 0;
    return mb_elapsed(_lastRequestAt, mb_millis());
}

void ModbusServer::poll() {
    if (_config.failsafeTimeout == 0) return;
    if (!_heardFromMaster) return;   // Never talked to; nothing to lose contact with

    if (mb_elapsed(_lastRequestAt, mb_millis()) < _config.failsafeTimeout) {
        _failsafeApplied = false;
        return;
    }

    // Applied ONCE per loss of contact, not on every poll. Driving an output to
    // its safe state repeatedly is harmless, but raising the handler fifty
    // times a second would bury the one event that mattered.
    if (_failsafeApplied) return;
    _failsafeApplied = true;

    const uint16_t driven = _map.applyFailsafe();
    for (size_t i = 0; i < _unitMaps.size(); i++) {
        _unitMaps[i].map->applyFailsafe();
    }

    if (driven && _eventHandler) {
        _eventHandler(_eventContext, ModbusEvent::FailsafeApplied,
                      _config.unitId, "the master has gone quiet");
    }
}

bool ModbusServer::handleAdu(const ModbusAdu& adu) {
    if (!addressedToUs(adu.unit)) {
        // On a serial line every device hears every frame. Answering one
        // addressed elsewhere would put two responses on the bus and corrupt
        // both, so this is a normal and frequent non-event.
        _forOthers++;
        return false;
    }

    _lastRequestAt = mb_millis();
    _heardFromMaster = true;
    _failsafeApplied = false;

    const bool broadcast = (adu.unit == MB_ADDRESS_BROADCAST);

    // A gateway's per-unit map, or ours. Swapped in around the call rather than
    // held, so one engine serves every unit and there is one place that knows
    // how a request is processed.
    ModbusDataMap& target = mapFor(adu.unit);
    ModbusServerEngine* engine = &_engine;
    ModbusServerEngine perUnit(target);
    if (&target != &_map) {
        perUnit.setIdentity(_engine.identity());
        perUnit.setSerialLine(_config.serialLine);
        engine = &perUnit;
    }

    const size_t length = engine->handle(adu.unit, broadcast, adu.pdu, adu.pduLen,
                                         _response, sizeof _response);

    if (length == 0) {
        // Silence is a legitimate answer on Modbus: a broadcast, a request in
        // listen-only mode, or one we could not make sense of. It is not an
        // error and must not be reported as one.
        return true;
    }

    ModbusAdu response;
    response.unit = adu.unit;
    response.transactionId = adu.transactionId;   // Echoed, for TCP
    response.pdu = _response;
    response.pduLen = (uint16_t)length;

    if (_link.send(response)) {
        _answered++;
    } else if (_eventHandler) {
        _eventHandler(_eventContext, ModbusEvent::LinkDown, adu.unit,
                      "a response could not be sent");
    }
    return true;
}
